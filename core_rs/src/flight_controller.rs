#![allow(dead_code)]
#![allow(unused_imports)]
use rclrs::*;
use std::time::{Duration, Instant, SystemTime};
use std::sync::{Arc, Mutex, RwLock};
use std::thread::sleep;
use tokio::sync::Mutex as AsyncMutex;
use tokio::sync::watch;
use anyhow::{anyhow, Result, Context as AnyhowContext}; // context和rclrs上下文context冲突
// ros2相关
use geometry_msgs::msg::PoseStamped;
use geometry_msgs::msg::TwistStamped;
use mavros_msgs::{
    msg::State,
    srv::{CommandLong, CommandBool, CommandBool_Request, SetMode, SetMode_Request},
};
// 自定义包
use ros2_tools::msg::LidarPose;
// crate
use crate::quadcopter::{*};
use crate::trajectory::{*};

const DEFAULT_POS_CHECK_DISTANCE: f64 = 0.15;

// 飞行控制器结构体
pub struct FlightController {
    node: Arc<Node>,
    context: Context,
    executor: Arc<Mutex<Executor>>,
    // 订阅组
    lidar_pos: Arc<Mutex<LidarPose>>,
    lidar_sub: Arc<Subscription<LidarPose>>,
    current_state: Arc<Mutex<State>>,
    state_sub: Arc<Subscription<State>>,
    // 发布组
    pos_pub: Arc<Publisher<PoseStamped>>,
    vel_pub: Arc<Publisher<TwistStamped>>,
    // mavros客户端组
    arming_client: Arc<ClientState<CommandBool>>,
    command_client: Arc<ClientState<CommandLong>>,
    set_mode_client: Arc<ClientState<SetMode>>,
}

// 飞行控制器实现
impl FlightController {
    pub fn new(self_pos_mut: Arc<Mutex<SelfPos>>) -> Result<Self> {
        let context = Context::default_from_env()
            .context("上下文创建失败")?;

        let executor = Arc::new(Mutex::new(context.create_basic_executor()));
         
        let node: Arc<Node> = executor.lock()
            .map_err(|e| anyhow!("获取 executor 锁失败: {}", e))?
            .create_node("flight_ctrl_node")
            .context("无法创建飞行控制节点")?
            .into();

        let lidar_pos = Arc::new(Mutex::new(LidarPose::default()));
        let lidar_pos_mut = Arc::clone(&lidar_pos);
        let lidar_sub = node
            .create_subscription::<ros2_tools::msg::LidarPose, _>(
                "lidar_data".qos(QoSProfile::default().best_effort()),
                move |msg: LidarPose| {
                    *lidar_pos_mut.lock().unwrap() = msg.clone();
                    // 更新自身位姿
                    let mut self_pos_mut = self_pos_mut.lock().unwrap();
                    *self_pos_mut = SelfPos {
                        x : msg.x, 
                        y : msg.y, 
                        z : msg.z, 
                        roll : msg.roll, 
                        pitch : msg.pitch, 
                        yaw : msg.yaw, 
                        ..Default::default() 
                    };
                }
            )
        .context("雷达订阅丢失")?
        .into();

        let current_state = Arc::new(Mutex::new(State::default()));
        let current_state_mut = Arc::clone(&current_state);
        let state_sub = node
            .create_subscription::<mavros_msgs::msg::State, _>(
                "mavros/state", 
                move |msg: State| {
                    *current_state_mut.lock().unwrap() = msg;
                }
            )
            .context("mavros状态订阅丢失")?
            .into();

        let pos_pub = node
            .create_publisher::<geometry_msgs::msg::PoseStamped>("mavros/setpoint_position/local")
            .context("pos_pub创建失败")?
            .into();
        let vel_pub = node
            .create_publisher::<geometry_msgs::msg::TwistStamped>("mavros/setpoint_velocity/cmd_vel")
            .context("vel_pub创建失败")?
            .into();
        let arming_client = node
            .create_client::<CommandBool>("mavros/cmd/arming")
            .context("arming_client创建失败")?;
        let command_client = node
            .create_client::<CommandLong>("mavros/cmd/command")
            .context("command_client创建失败")?;
        let set_mode_client = node
            .create_client::<SetMode>("mavros/set_mode")
            .context("set_mode_client创建失败")?;
        
        println!("飞行控制器初始化完成");

        Ok(FlightController {
            node, context, executor,
            lidar_pos, lidar_sub,
            current_state, state_sub,
            pos_pub, vel_pub,
            arming_client, command_client, set_mode_client,
        })
    }

    // 预飞行检查
    pub async fn pre_flight_checks_loop(&mut self) -> Result<()> {
        // 等待飞控连接
        while self.context.ok() && !self.current_state.lock().unwrap().connected {
            self.executor
                .lock()
                .unwrap()
                .spin(SpinOptions::default().timeout(Duration::from_millis(20)));
            println!("等待飞控连接...");
            tokio::time::sleep(Duration::from_millis(20)).await;
        }

        // 起飞预发布
        let mut simp = Target::new(0.0, 0.0, 0.5, 0.0);
        for _ in 0..20 {
            simp.set_time_now();
            self.pos_pub.publish(simp.get_pose().clone())?;
            self.executor
                .lock()
                .unwrap()
                .spin(SpinOptions::default().timeout(Duration::from_millis(20)));
            tokio::time::sleep(Duration::from_millis(20)).await;
        }
        println!("起飞预发布完成");

        // 初始化service请求
        let mut offb_set_mode = SetMode_Request::default();
        offb_set_mode.custom_mode = "OFFBOARD".to_string();

        let mut arm_cmd = CommandBool_Request::default();
        arm_cmd.value = true;

        // 等待service就绪
        self.arming_client.notify_on_service_ready().await?;
        self.set_mode_client.notify_on_service_ready().await?;

        // mavros状态检测与服务调用
        let mut last_request = Instant::now();
        while self.context.ok() {
            simp.set_time_now();
            self.pos_pub.publish(simp.get_pose().clone())?;

            let state = self.current_state.lock().unwrap().clone();
            if !state.armed && (Instant::now() - last_request > Duration::from_secs(1)) {
                let res: mavros_msgs::srv::CommandBool_Response = self
                        .arming_client
                        .call(&arm_cmd)?
                        .await
                        .context("Arming service wait failed")?;
                if res.success { 
                    println!("arming..."); 
                }
                last_request = Instant::now();
            } else if state.mode != "OFFBOARD" && (Instant::now() - last_request > Duration::from_secs(1)) {
                let res: mavros_msgs::srv::SetMode_Response = self
                    .set_mode_client
                    .call(&offb_set_mode)?
                    .await
                    .context("SetMode service wait failed")?;
                if res.mode_sent {
                    println!("armed and OFFBOARDING...");
                }
                last_request = Instant::now();
            } else if state.armed && state.mode == "OFFBOARD" {
                println!("armed and OFFBOARD success!");
                break;
            }

            self.executor
                .lock()
                .unwrap()
                .spin(SpinOptions::default().timeout(Duration::from_millis(20)));
            tokio::time::sleep(Duration::from_millis(20)).await;
        }

        // 阻塞执行起飞点
        let _ = self.fly_to_target_sync(&mut simp);
        Ok(())
    }

    // target定点移动，同步版本
    pub fn fly_to_target_sync(&mut self, target: &mut Target) -> Result<()> {
        while self.context.ok() && !self.pos_check(target) {
            target.set_time_now();
            self.pos_pub.publish(target.get_pose().clone())
                .context("发布目标点失败")?;
            self.executor
                .lock()
                .map_err(|e| anyhow!("target模式获取executor锁失败: {}", e))?
                .spin(SpinOptions::default().timeout(Duration::from_millis(20)));
            sleep(Duration::from_millis(20));
        }
        Ok(())
    }

    // target定点移动，异步版本
    pub async fn fly_to_target(&mut self, target: Arc<AsyncMutex<Target>>, async_rx: Arc<RwLock<watch::Receiver<bool>>>) -> Result<()> {
        let mut target = target.lock().await;
        while self.context.ok() && !self.pos_check(&mut *target) {
            // 发布目标点
            target.set_time_now();
            self.pos_pub
                .publish(target.get_pose().clone())
                .context("异步target发布失败")?;

            self.executor
                .lock()
                .map_err(|e| anyhow!("获取executor锁失败: {}", e))?
                .spin(SpinOptions::default().timeout(Duration::from_millis(20)));

            // 异步等待
            // 注意，rx是接收器不能带锁穿越await
            let mut rx = {
                let rx = async_rx.write()
                    .map_err(|e| anyhow!("获取异步rx锁失败: {}", e))?;
                rx.clone()
            };
            tokio::select! {
                // 正常控制周期
                _ = tokio::time::sleep(Duration::from_millis(20)) => {}
                // 中断检查
                _ = rx.changed() => {
                    if *async_rx.read().map_err(|e| anyhow!("获取异步rx读锁失败: {}", e))?.borrow() {
                        println!("任务中断：挂起 fly_to_target");
                        break;
                    }
                }
            }
        }
        Ok(())
    }

    // 自身位置检查，DEFAULT_POS_CHECK_DISTANCE为默认值0.15
    fn pos_check(&self, target: &mut Target) -> bool {
        if target.reached { return true; }
        let lidar_pos = self.lidar_pos.lock().unwrap();
        let distance = ((lidar_pos.x - target.get_x()).powi(2) + 
                        (lidar_pos.y - target.get_y()).powi(2) + 
                        (lidar_pos.z - target.get_z()).powi(2))
                        .sqrt();
        let dyaw = lidar_pos.yaw - target.get_yaw().abs();
        target.reached = distance < DEFAULT_POS_CHECK_DISTANCE && dyaw < 0.1;
        target.reached
    }

    // 严格位置检查
    fn pos_check_strict(&self, target: &mut Target, distance_x: f64, distance_y: f64, distance_z: f64) -> bool {
        if target.reached { return true; }
        let lidar_pos = self.lidar_pos.lock().unwrap(); 
        let dx = lidar_pos.x - target.get_x();
        let dy = lidar_pos.y - target.get_y();
        let dz = lidar_pos.z - target.get_z();
        let dyaw = lidar_pos.yaw - target.get_yaw().abs();
        target.reached = dx < distance_x && dy < distance_y && dz < distance_z && dyaw < 0.1;
        target.reached
    }

    // velocity速度飞行，单次发布
    pub fn fly_by_velocity(&self, velocity: &mut Velocity) -> Result<()> {
        velocity.set_time_now();
        self.vel_pub
            .publish(velocity.get_twist().clone())
            .context("velocity发布失败")?;
        Ok(())
    }

    // velocity速度飞行，持续时间duration，异步监听
    pub async fn fly_by_vel_duration(&mut self, velocity: &mut Velocity, duration: f64, async_rx: Arc<RwLock<watch::Receiver<bool>>>) -> Result<()> {
        let start_time = SystemTime::now();
        let duration = Duration::from_secs_f64(duration);
        let lidar_pos = self.lidar_pos.lock().unwrap();
        let start_altitude = lidar_pos.z;

        while SystemTime::now().duration_since(start_time).unwrap() < duration {
            // 修改z轴反馈
            if (start_altitude - lidar_pos.z).abs() > 0.1 { velocity.set_vz(start_altitude - lidar_pos.z); }
            let _ = self.fly_by_velocity(velocity);
            self.executor
                .lock()
                .map_err(|e| anyhow!("velocity模式获取executor锁失败: {}", e))?
                .spin(SpinOptions::default().timeout(Duration::ZERO));

            // 异步等待
            // 这里需要注意，rx是接收器不能带锁穿越await
            let mut rx = {
                let rx = async_rx.write().map_err(|e| anyhow!("velocity模式获取异步rx锁失败: {}", e))?;
                rx.clone()
            };
            tokio::select! {
                // 正常控制周期
                _ = tokio::time::sleep(Duration::from_millis(20)) => {}
                // 中断检查
                _ = rx.changed() => {
                    if *async_rx.read().map_err(|e| anyhow!("velocity模式获取异步rx锁失败: {}", e))?.borrow() {
                        println!("任务中断：挂起 fly_by_vel_duration");
                        break;
                    }
                }
            }
        }
        Ok(())
    }

    // 异步路径航点飞行
    pub async fn fly_by_path(&mut self, path: Arc<AsyncMutex<Path>>, async_rx: Arc<RwLock<watch::Receiver<bool>>>) {
        while self.context.ok() {
            let mut path = path.lock().await;
            if path.done().await {
                println!("航点已全部执行完毕");
                break;
            }

            if let Some(waypoint) = path.get_current_waypoint().await {
                let index = path.get_index().await;
                match self.fly_to_target(waypoint, async_rx.clone()).await {
                    Ok(_) => {
                        println!("到达第{}航点", index+1);
                        path.advance().await // 航点自增
                    },
                    Err(e) => {
                        println!("第{}航点失败, 错误信息:{}", index+1, e);
                        let _ = self.land();
                        panic!("主动降落退出程序！")
                    }
                }
            }
        }
    }

    // 降落AUTO.LAND模式
    pub async fn auto_land(&mut self) -> Result<()> {
        let mut offb_set_mode = SetMode_Request::default();
        offb_set_mode.custom_mode = "AUTO.LAND".to_string();
        let mut last_request = Instant::now();
        while self.context.ok() {
            let state = self.current_state
                .lock()
                .map_err(|e| anyhow!("自动降落获取mavros状态锁失败: {}", e))?
                .clone();

            if state.mode != "AUTO.LAND" && (Instant::now() - last_request > Duration::from_secs(1)) {
                let res: mavros_msgs::srv::SetMode_Response = self.set_mode_client
                    .call(&offb_set_mode)?
                    .await
                    .context("SetMode AUTOLAND failed")?;
                    if res.mode_sent {
                        println!("开始降落...");
                    }
                last_request = Instant::now();
            } else if state.mode == "AUTO.LAND" {
                println!("正在降落");
            } else if state.mode == "LANDED" || 
                      state.mode == "STANDBY" {
                println!("降落完成");
                break;
            }

            self.executor
                .lock()
                .map_err(|e| anyhow!("自动降落获取executor锁失败: {}", e))?
                .spin(SpinOptions::default().timeout(Duration::from_millis(20)));
            tokio::time::sleep(Duration::from_millis(20)).await;
        }
        Ok(())
    }

    // 硬着陆，以当前坐标降落
    pub fn land(&mut self) -> Result<()> {
        let lidar_pos = self.lidar_pos.lock().unwrap();
        let mut land_point= Target::new(lidar_pos.x, lidar_pos.y, 0.05, lidar_pos.yaw);
        drop(lidar_pos); // 释放锁，否则无法阻塞式降落
        match self.fly_to_target_sync(&mut land_point) {
            Ok(_) => println!("硬着陆ing..."),
            Err(e) => return Err(anyhow!("硬着陆失败: {}", e)),
        };
        Ok(())
    }

    // *已弃用* 预飞行检查，旧版client写法，25/7/4更新
    /*
    pub fn pre_flight_checks_loop(&mut self) -> Result<(), RclrsError> {
        // 等待飞控连接
        while self.context.ok() && !self.current_state.lock().unwrap().connected {
            self.executor.lock().unwrap().spin(SpinOptions::default().timeout(Duration::from_millis(20)));
            println!("等待飞控连接...");
            sleep(Duration::from_millis(20));
        }

        // 起飞预发布
        let mut simp = Target::new(0.0, 0.0, 0.5, 0.0);
        for _ in 0..20 {
            simp.set_time_now();
            self.pos_pub.publish(simp.get_pose().clone()).unwrap();
            self.executor.lock().unwrap().spin(SpinOptions::default().timeout(Duration::from_millis(20)));
            sleep(Duration::from_millis(20));
        }
        println!("起飞预发布完成");

        let mut offb_set_mode = SetMode_Request::default();
        offb_set_mode.custom_mode = "OFFBOARD".to_string();

        let mut arm_cmd = CommandBool_Request::default();
        arm_cmd.value = true;

        // mavros状态检查
        let mut last_request = Instant::now();
        while self.context.ok() {
            simp.set_time_now();
            self.pos_pub.publish(simp.get_pose().clone()).unwrap();

            if !self.current_state.lock().unwrap().armed && (Instant::now() - last_request > Duration::from_secs(1)) {
                self.arming_client.async_send_request_with_callback(&arm_cmd, |res| {
                    if res.success { println!("arming..."); }
                })?;
                last_request = Instant::now();
            } else if self.current_state.lock().unwrap().mode != "OFFBOARD" && (Instant::now() - last_request > Duration::from_secs(1)) {
                self.set_mode_client.async_send_request_with_callback(&offb_set_mode, |res| {
                    if res.mode_sent { println!("armed and OFFBOARDING..."); }
                })?;
                last_request = Instant::now();
            } else if self.current_state.lock().unwrap().armed && self.current_state.lock().unwrap().mode == "OFFBOARD" {
                println!("armed and OFFBOARD success!");
                break;
            }
            self.executor.lock().unwrap().spin(SpinOptions::default().timeout(Duration::from_millis(20)));
            sleep(Duration::from_millis(20));
        }
        // 起飞点
        self.fly_to_target_sync(&mut simp);
        Ok(())
    }
    */
}