use rclrs::*;
use std::{env, sync::Arc, thread, time::Duration};
use geometry_msgs::msg::PoseStamped;
use geometry_msgs::msg::TwistStamped;
use mavros_msgs::msg::State;
use mavros_msgs::srv::CommandBool;
use mavros_msgs::srv::CommandLong;
use mavros_msgs::srv::SetMode;

use ros2_tools::msg::LidarPose;
use vision_msgs::msg::Vision;

use crate::{flight_controller::FlightController, target::Target, velocity::Velocity};

// 飞行器结构体
struct QuadCopter {
    flight_ctrl: Arc<Mutex<FlightController>>,
    rate: Rate,

    lidar_pos: Arc<Mutex<LidarPose>>,
    lidar_sub: Arc<Subscription<LidarPose>>,
    current_state: Arc<Mutex<State>>,
    state_sub: Arc<Subscription<State>>,
    vision_msg: Arc<Mutex<Vision>>,
    vision_sub: Arc<Subscription<Vision>>,

    pos_pub: Arc<Publisher<PoseStamped>>,
    vel_pub: Arc<Publisher<TwistStamped>>,

    arming_client: Arc<Client<CommandBool>>,
    command_client: Arc<Client<CommandLong>>,
    set_mode_client: Arc<Client<SetMode>>,

    pub x: f32, pub y: f32, pub z: f32, pub yaw: f32,
    pub vx: f32, pub vy: f32, pub vz: f32,
    pub ax: f32, pub ay: f32, pub az: f32,
    pub roll: f32, pub pitch: f32,
    pub dr: f32, pub dp: f32,
}

// 飞行器内置函数
impl QuadCopter {
    // 构造函数
    fn new(context: &context) -> Self {
        let node = crate_node(context, "quad_node")?;
        let rate = node.create_rate(20.0)?;

        let lidar_pos = Arc::new(Mutex::new(LidarPose::default()));
        let lidar_sub = node.create_subscription::<ros2_tools::msg::LidarPose>("lidar_pose", 10, lidar_pose_cb)?;
        let current_state = Arc::new(Mutex::new(State::default()));
        let state_sub = node.create_subscription::<mavros_msgs::msg::State>("mavros/state", 10, state_cb)?;
        let vision_msg = Arc::new(Mutex::new(Vision::default()));
        let vision_sub = node.create_subscription::<vision_msgs::msg::Vision>("vision", 10, vision_sub_cb)?;

        let qos = QosProfile::default().with_history(rclrs::QosHistoryPolicy::KeepLast(10))?; // 手动设置qos队列深度10
        let pos_pub = node.crate_publisher::<geometry_msgs::msg::PoseStamped>("mavros/setpoint_position/local", qos)?;
        let vel_pub = node.create_publisher::<geometry_msgs::msg::TwistStamped>("mavros/setpoint_velocity/cmd_vel", qos)?;
        
        let arming_client = node.create_client::<CommandBool>("mavros/cmd/arming")?;
        let command_client = node.create_client::<CommandLong>("mavros/cmd/command")?;
        let set_mode_client = node.create_client::<SetMode>("mavros/set_mode")?;
        println!("飞行器初始化完成");

        Ok(QuadCopter {
            rate,
            flight_ctrl: None,
            lidar_pos, lidar_sub,
            current_state, state_sub,
            vision_msg, vision_sub,
            pos_pub, vel_pub,
            arming_client, command_client, set_mode_client,
            x: 0.0, y: 0.0, z: 0.0, yaw: 0.0,
            vx: 0.0, vy: 0.0, vz: 0.0,
            ax: 0.0, ay: 0.0, az: 0.0,
            roll: 0.0, pitch: 0.0,
            dr: 0.0, dp: 0.0,
        })
    }

    // 初始化quad节点控制流程
    pub fn quad_init(&self) {
        self.flight_ctrl_init();
        self.start_spin_thread();
        self.pre_flight_checks_loop();
        self.main_loop();
    }

    fn flight_ctrl_init(&mut self) {
        let quad_arc = Arc::new(Mutex::new(self)); // 这里需要调整所有权
        
        self.flight_ctrl = Some(Arc::new(Mutex::new(
            FlightController::new(quad_arc)
        )));
        info!("飞行控制类初始化完成");
    }

    // 单开线程处理回调
    fn start_spin_thread(&self) {
        thread::spawn(|| {
            rosrust::spin();
        });
    }

    // 预飞行检查
    fn pre_flight_checks_loop(&self) {
        let mut offb_set_mode = SetMode::Request::default();
        offb_set_mode.custom_mode = "OFFBOARD".to_string();

        let mut arm_cmd = CommandBool::Request::default();
        arm_cmd.value = true;

        // Publish takeoff position
        let mut simp = Target::new(0.0, 0.0, 0.5, 0.0);
        for _ in 0..20 {
            simp.set_time(rosrust::now());
            self.pos_pub.send(simp.get_pose().clone()).unwrap();
            self.rate.sleep();
        }

        let mut last_request = rosrust::now();
        loop {
            simp.set_time(rosrust::now());
            self.pos_pub.send(simp.get_pose().clone()).unwrap();

            let state = self.current_state.lock().unwrap();
            if !state.armed && (rosrust::now() - last_request > Duration::from_secs(1)) {
                if let Ok(_) = self.arming_client.call(&arm_cmd) {
                    info!("arming...");
                }
                last_request = rosrust::now();
            } else if state.mode != "OFFBOARD" && (rosrust::now() - last_request > Duration::from_secs(1)) {
                if let Ok(_) = self.set_mode_client.call(&offb_set_mode) {
                    info!("armed and OFFBOARDING...");
                }
                last_request = rosrust::now();
            } else if state.armed && state.mode == "OFFBOARD" {
                info!("armed and OFFBOARD success!");
                self.flight_ctrl.lock().unwrap().fly_to_target(&mut simp);
                break;
            }
            self.rate.sleep();
        }
    }

    // 主循环
    fn main_loop(&self) {
        let mut flag = 0;
        let default_altitude = 1.5;
        let mut is_complete_cast = false;
        let mut first_point = Target::new(0.0, 0.0, 1.5, 0.0);
        let mut tar1 = Target::new(0.0, 0.0, 0.0, 0.0);
        let mut vel1 = Velocity::linear(0.1, 0.0, 0.0);
        let mut vel2 = Velocity::linear(0.15, 0.0, 0.0);

        loop {
            match flag {
                0 => {
                    self.flight_ctrl.lock().unwrap().fly_to_target(&mut first_point);
                    info!("到达指定高度");
                    flag = 1;
                    info!("前进");
                }
                1 => {
                    self.flight_ctrl.lock().unwrap().fly_by_velocity(&mut vel1);
                    let vision = self.vision_msg.lock().unwrap();
                    if vision.is_line_detected {
                        info!("发现直线");
                        flag = 2;
                        info!("巡线");
                    }
                }
                2 => {
                    let mut vision = self.vision_msg.lock().unwrap();
                    vel2.set_vy(vision.lateral_error / -1000.0);
                    vel2.set_vyaw(vision.angle_error / 5.0);

                    if default_altitude - self.z > 0.05 {
                        vel2.set_vz(default_altitude - self.z);
                    } else if default_altitude - self.z < -0.05 {
                        vel2.set_vz(default_altitude - self.z);
                    }

                    self.flight_ctrl.lock().unwrap().fly_by_velocity(&mut vel2);

                    if vision.is_square_detected && !is_complete_cast {
                        tar1.set_x(self.x);
                        tar1.set_y(self.y);
                        tar1.set_z(self.z);
                        tar1.set_yaw(self.yaw);
                        info!(
                            "发现形状, 记录位置: x={}, y={}, z={}, yaw={}",
                            self.x, self.y, self.z, self.yaw
                        );
                        flag = 3;
                        info!("进入校准");
                    }

                    if vision.is_circle_detected && is_complete_cast {
                        info!("发现降落区域");
                        flag = 5;
                        info!("进入校准降落");
                    }
                }
                3 => {
                    let vision = self.vision_msg.lock().unwrap();
                    vel2.set_vx(vision.center_x1_error / -1000.0);
                    vel2.set_vy(vision.center_y1_error / -1000.0);
                    vel2.set_vyaw(0.0);

                    if default_altitude - self.z > 0.05 {
                        vel2.set_vz(default_altitude - self.z);
                    } else if default_altitude - self.z < -0.05 {
                        vel2.set_vz(default_altitude - self.z);
                    }

                    self.flight_ctrl.lock().unwrap().fly_by_velocity(&mut vel2);

                    if vision.center_x1_error.abs() < 20.0 && vision.center_y1_error.abs() < 20.0 {
                        info!("投掷");
                        is_complete_cast = true;
                        flag = 4;
                        info!("返回巡线");
                    }
                }
                4 => {
                    self.flight_ctrl.lock().unwrap().fly_to_target(&mut tar1);
                    vel2.set_vx(0.15);
                    info!("继续巡线");
                    flag = 2;
                }
                5 => {
                    let vision = self.vision_msg.lock().unwrap();
                    vel2.set_vx(vision.center_x2_error / -1000.0);
                    vel2.set_vy(vision.center_y2_error / -1000.0);
                    vel2.set_vyaw(0.0);

                    if default_altitude - self.z > 0.05 {
                        vel2.set_vz(0.03);
                    } else if default_altitude - self.z < -0.05 {
                        vel2.set_vz(-0.03);
                    }

                    self.flight_ctrl.lock().unwrap().fly_by_velocity(&mut vel2);

                    if vision.center_x2_error.abs() < 20.0 && vision.center_y2_error.abs() < 20.0 {
                        info!("降落");
                        vel2.set_vx(0.0);
                        vel2.set_vy(0.0);
                        vel2.set_vz(-0.2);
                        self.flight_ctrl
                            .lock()
                            .unwrap()
                            .fly_by_vel_duration(&mut vel2, 5.0);
                        info!("降落完成");
                        flag = 6;
                    }
                }
                _ => break,
            }
            self.rate.sleep();
        }
    }

    // lidar数据回调
    fn lidar_pose_cb(&mut self, msg: LidarPose) {
        *self.lidar_pos.lock().unwrap() = msg.clone();
        self.x = msg.x;
        self.y = msg.y;
        self.z = msg.z;
        self.yaw = msg.yaw;
    }

    // mavros状态回调
    fn state_cb(&mut self, msg: State) {
        *self.current_state.lock().unwrap() = msg;
    }

    // vision数据回调
    fn vision_sub_cb(&mut self, msg: Vision) {
        *self.vision_msg.lock().unwrap() = msg;
    }
}

// 主程序入口
fn main() {
    rosrust::init("quad_node");
    let quad = Quadcopter::new();
    quad.quad_init();
}