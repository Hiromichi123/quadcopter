use rclrs::*;
use std::fmt;
use std::thread::sleep;
use std::time::{Duration, Instant, SystemTime};
use std::sync::{Arc, Mutex, RwLock};
use tokio::sync::Mutex as AsyncMutex;
use tokio::sync::watch;
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
#[allow(unused_imports)]
use crate::quadcopter::{*};
#[allow(unused_imports)]
use crate::trajectory::{*};

const DEFAULT_POS_CHECK_DISTANCE: f64 = 0.15;

// 飞行控制器结构体
#[allow(dead_code)]
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
    arming_client: Arc<Client<CommandBool>>,
    command_client: Arc<Client<CommandLong>>,
    set_mode_client: Arc<Client<SetMode>>,
}

// 实现 Debug trait
impl fmt::Debug for FlightController {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        f.debug_struct("FlightController")
            .field("node", &self.node)
            .field("context", &format_args!("上下文错误"))
            .field("executor", &format_args!("节点执行器错误"))
            .finish()
    }
}

// 飞行控制器实现
#[allow(dead_code)]
impl FlightController {
    pub fn new(self_pos_mut: Arc<Mutex<SelfPos>>) -> Result<Self, RclrsError> {
        let context = Context::default_from_env().unwrap();
        let executor = Arc::new(Mutex::new(context.create_basic_executor()));
        let node = executor.lock().unwrap().create_node("flight_ctrl_node").unwrap();

        let lidar_pos = Arc::new(Mutex::new(LidarPose::default()));
        let lidar_pos_mut = Arc::clone(&lidar_pos);
        let lidar_sub = node.create_subscription::<ros2_tools::msg::LidarPose, _>("lidar_data".qos(QoSProfile::default().best_effort()),
            move |msg: LidarPose| {
                *lidar_pos_mut.lock().unwrap() = msg.clone();
                let mut self_pos_mut = self_pos_mut.lock().unwrap();
                *self_pos_mut = SelfPos {
                    x : msg.x, 
                    y : msg.y, 
                    z : msg.z, 
                    roll : msg.roll, 
                    pitch : msg.pitch, 
                    yaw : msg.yaw, 
                    ..Default::default() 
                }
            }
        ).unwrap();

        let current_state = Arc::new(Mutex::new(State::default()));
        let current_state_mut = Arc::clone(&current_state);
        let state_sub = node.create_subscription::<mavros_msgs::msg::State, _>("mavros/state", 
            move |msg: State| {
                *current_state_mut.lock().unwrap() = msg;
            }
        ).unwrap();

        let pos_pub = node.create_publisher::<geometry_msgs::msg::PoseStamped>("mavros/setpoint_position/local").unwrap();
        let vel_pub = node.create_publisher::<geometry_msgs::msg::TwistStamped>("mavros/setpoint_velocity/cmd_vel").unwrap();
        
        let arming_client = node.create_client::<CommandBool>("mavros/cmd/arming").unwrap();
        let command_client = node.create_client::<CommandLong>("mavros/cmd/command").unwrap();
        let set_mode_client = node.create_client::<SetMode>("mavros/set_mode").unwrap();
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

        // mavros服务响应
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

    // target定点移动，同步版本
    pub fn fly_to_target_sync(&mut self, target: &mut Target) {
        while self.context.ok() && !self.pos_check(target) {
            target.set_time_now();
            self.pos_pub.publish(target.get_pose().clone()).unwrap();
            self.executor.lock().unwrap().spin(SpinOptions::default().timeout(Duration::from_millis(20)));
            sleep(Duration::from_millis(20));
        }
    }

    // target定点移动，异步版本
    pub async fn fly_to_target(&mut self, target: Arc<AsyncMutex<Target>>, async_rx: Arc<RwLock<watch::Receiver<bool>>>) {
        let mut target = target.lock().await;
        while self.context.ok() && !self.pos_check(&mut *target) {
            // 发布目标点
            target.set_time_now();
            self.pos_pub.publish(target.get_pose().clone()).unwrap();
            self.executor.lock().unwrap().spin(SpinOptions::default().timeout(Duration::from_millis(20)));

            // 异步等待
            // 这里需要注意，rx是接收器，不能带锁穿越await
            let mut rx = {
                let rx = async_rx.write().unwrap();
                rx.clone()
            };
            tokio::select! {
                // 正常控制周期
                _ = tokio::time::sleep(Duration::from_millis(20)) => {}
                // 中断检查
                _ = rx.changed() => {
                    if *async_rx.read().unwrap().borrow() {
                        println!("任务中断：挂起 fly_to_target");
                        break;
                    }
                }
            }
        }
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
    pub fn fly_by_velocity(&self, velocity: &mut Velocity) {
        velocity.set_time_now();
        self.vel_pub.publish(velocity.get_twist().clone()).unwrap();
    }

    // velocity速度飞行，持续时间duration，异步监听
    pub async fn fly_by_vel_duration(&mut self, velocity: &mut Velocity, duration: f64, async_rx: Arc<RwLock<watch::Receiver<bool>>>) {
        let start_time = SystemTime::now();
        let duration = Duration::from_secs_f64(duration);
        let lidar_pos = self.lidar_pos.lock().unwrap(); // 解锁
        let start_altitude = lidar_pos.z;

        while SystemTime::now().duration_since(start_time).unwrap() < duration {
            // 修改z轴反馈
            if (start_altitude - lidar_pos.z).abs() > 0.1 { velocity.set_vz(start_altitude - lidar_pos.z); }
            self.fly_by_velocity(velocity);
            self.executor.lock().unwrap().spin(SpinOptions::default().timeout(Duration::ZERO));

            // 异步等待
            // 这里需要注意，rx是接收器，不能带锁穿越await
            let mut rx = {
                let rx = async_rx.write().unwrap();
                rx.clone()
            };
            tokio::select! {
                // 正常控制周期
                _ = tokio::time::sleep(Duration::from_millis(20)) => {}
                // 中断检查
                _ = rx.changed() => {
                    if *async_rx.read().unwrap().borrow() {
                        println!("任务中断：挂起 fly_by_vel_duration");
                        break;
                    }
                }
            }
        }
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
                self.fly_to_target(waypoint, async_rx.clone()).await; // 异步等待飞行完成
                path.advance().await; // 航点自增
            }
        }
    }

    // 降落AUTO.LAND模式
    pub fn auto_land(&mut self) {
        let mut offb_set_mode = SetMode_Request::default();
        offb_set_mode.custom_mode = "AUTO.LAND".to_string();
        let mut last_request = Instant::now();
        while self.context.ok() {
            if self.current_state.lock().unwrap().mode != "AUTO.LAND" && (Instant::now() - last_request > Duration::from_secs(1)) {
                self.set_mode_client.async_send_request_with_callback(&offb_set_mode, |res| {
                    if res.mode_sent { println!("landing..."); }
                }).unwrap();
                last_request = Instant::now();
            } else if self.current_state.lock().unwrap().mode == "AUTO.LAND" {
                println!("降落成功");
                break;
            }
            self.executor.lock().unwrap().spin(SpinOptions::default().timeout(Duration::from_millis(20)));
            sleep(Duration::from_millis(20));
        }
    }

    // 硬着陆，以当前坐标降落
    pub fn land(&mut self) {
        let lidar_pos = self.lidar_pos.lock().unwrap();
        let mut land_point= Target::new(lidar_pos.x, lidar_pos.y, 0.05, lidar_pos.yaw);
        drop(lidar_pos);
        println!("landing...");
        self.fly_to_target_sync(&mut land_point);
    }
}