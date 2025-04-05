use rclrs::*;
use std::{env, thread};
use std::time::{Duration, Instant};
use std::sync::{Arc, Mutex};
// ros2相关组件
use geometry_msgs::msg::PoseStamped;
use geometry_msgs::msg::TwistStamped;
use mavros_msgs::msg::State;
use mavros_msgs::srv::CommandBool;
use mavros_msgs::srv::CommandLong;
use mavros_msgs::srv::SetMode;
// 自定义包组件
use ros2_tools::msg::LidarPose;
use vision_py::msg::Vision;

mod flight_controller;
mod target;
mod velocity;
use crate::{flight_controller::FlightController, target::Target, velocity::Velocity};

// 飞行器结构体
struct Quadcopter {
    node: Node,
    flight_ctrl: Arc<Mutex<FlightController>>,

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

    pub x: f64, pub y: f64, pub z: f64, pub yaw: f64,
    pub vx: f64, pub vy: f64, pub vz: f64,
    pub ax: f64, pub ay: f64, pub az: f64,
    pub roll: f64, pub pitch: f64,
    pub dr: f64, pub dp: f64,
}

// 飞行器内置函数
impl Quadcopter {
    // 构造函数
    fn new() -> Self {
        let context = Context::default_from_env()?;
        let mut executor = context.create_basic_executor();
        let node = executor.create_node("quad_node")?;

        let lidar_pos = Arc::new(Mutex::new(LidarPose::default()));
        let lidar_pos_mut = Arc::clone(&lidar_pos);
        let lidar_sub = node.create_subscription::<ros2_tools::msg::LidarPose>("lidar_pose", 10, 
            move |msg: LidarPose| {
                *lidar_pos_mut.lock().unwrap() = Some(msg);
                self.x = msg.x;
                self.y = msg.y;
                self.z = msg.z;
                self.yaw = msg.yaw;
            }
        )?;

        let current_state = Arc::new(Mutex::new(State::default()));
        let current_state_mut = Arc::clone(&current_state);
        let state_sub = node.create_subscription::<mavros_msgs::msg::State>("mavros/state", 10, 
            move |msg: State| {
                *current_state_mut.lock().unwrap() = Some(msg);
            }
        )?;

        let vision_msg = Arc::new(Mutex::new(Vision::default()));
        let vision_msg_mut = Arc::clone(&vision_msg);
        let vision_sub = node.create_subscription::<vision_msgs::msg::Vision>("vision", 10, 
            move |msg: Vision| {
                *vision_msg_mut.lock().unwrap() = Some(msg);
            }
        )?;

        let pos_pub = node.create_publisher::<geometry_msgs::msg::PoseStamped>("mavros/setpoint_position/local", 10)?;
        let vel_pub = node.create_publisher::<geometry_msgs::msg::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10)?;
        
        let arming_client = node.create_client::<CommandBool>("mavros/cmd/arming")?;
        let command_client = node.create_client::<CommandLong>("mavros/cmd/command")?;
        let set_mode_client = node.create_client::<SetMode>("mavros/set_mode")?;
        println!("飞行器初始化完成");

        Ok(Quadcopter {
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
        self.pre_flight_checks_loop();
        self.main_loop();
    }

    // 飞行控制类初始化
    fn flight_ctrl_init(&mut self) {
        let quad_arc = Arc::new(Mutex::new(self)); // 这里需要调整所有权
        
        self.flight_ctrl = Some(Arc::new(Mutex::new(
            FlightController::new(quad_arc)
        )));
        println!("飞行控制类初始化完成");
    }

    // 预飞行检查
    fn pre_flight_checks_loop(&self) {
        // 等待连接
        while context.ok() && !current_state.lock().unwrap().connected {
            executor.spin(SpinOptions::default().timeout(Duration::ZERO));
            sleep(Duration::from_secs(1));
        }
        let mut offb_set_mode = SetMode::Request::default();
        offb_set_mode.custom_mode = "OFFBOARD".to_string();

        let mut arm_cmd = CommandBool::Request::default();
        arm_cmd.value = true;

        // 起飞预发布
        let mut simp = Target::new(0.0, 0.0, 0.5, 0.0);
        for _ in 0..20 {
            simp.set_time(Instant::now());
            self.pos_pub.send(simp.get_pose().clone()).unwrap();
            executor.spin(SpinOptions::default().timeout(Duration::ZERO));
            sleep(Duration::from_secs(1));
        }

        let mut last_request = Instant::now();
        while context.ok() {
            simp.set_time(Instant::now());
            self.pos_pub.send(simp.get_pose().clone()).unwrap();
            let state = self.current_state.lock().unwrap();
            
            if !state.armed && (Instant::now() - last_request > Duration::from_secs(1)) {
                if let Ok(_) = self.arming_client.call(&arm_cmd) {
                    println!("arming...");
                }
                last_request = Instant::now();
            } else if state.mode != "OFFBOARD" && (Instant::now() - last_request > Duration::from_secs(1)) {
                if let Ok(_) = self.set_mode_client.call(&offb_set_mode) {
                    println!("armed and OFFBOARDING...");
                }
                last_request = Instant::now();
            } else if state.armed && state.mode == "OFFBOARD" {
                println!("armed and OFFBOARD success!");
                self.flight_ctrl.lock().unwrap().fly_to_target(&mut simp);
                break;
            }
            executor.spin(SpinOptions::default().timeout(Duration::ZERO));
            sleep(Duration::from_secs(1));
        }
    }

    // 主循环
    fn main_loop(&self) {
        let mut flag = 0;
        let default_altitude = 1.5;
        let mut is_complete_cast = false;
        let mut first_point = Target::new(0.0, 0.0, 1.5, 0.0);
        let mut tar1 = Target::new(0.0, 0.0, 0.0, 0.0);
        let mut vel1 = Velocity::new(0.1, 0.0, 0.0, 0.0, 0.0, 0.0);
        let mut vel2 = Velocity::new(0.15, 0.0, 0.0, 0.0, 0.0, 0.0);

        loop {
            match flag {
                0 => {
                    self.flight_ctrl.lock().unwrap().fly_to_target(&mut first_point);
                    println!("到达指定高度");
                    flag = 1;
                    println!("前进");
                }
                1 => {
                    self.flight_ctrl.lock().unwrap().fly_by_velocity(&mut vel1);
                    let vision = self.vision_msg.lock().unwrap();
                    if vision.is_line_detected {
                        println!("发现直线");
                        flag = 2;
                        println!("巡线");
                    }
                }
                2 => {
                    let mut vision = self.vision_msg.lock().unwrap();
                    vel2.set_vy((vision.lateral_error as f64) / -1000.0);
                    vel2.set_vyaw((vision.angle_error as f64) / 5.0);

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
                        println!(
                            "发现形状, 记录位置: x={}, y={}, z={}, yaw={}",
                            self.x, self.y, self.z, self.yaw
                        );
                        flag = 3;
                        println!("进入校准");
                    }

                    if vision.is_circle_detected && is_complete_cast {
                        println!("发现降落区域");
                        flag = 5;
                        println!("进入校准降落");
                    }
                }
                3 => {
                    let vision = self.vision_msg.lock().unwrap();
                    vel2.set_vx((vision.center_x1_error as f64) / -1000.0);
                    vel2.set_vy((vision.center_y1_error as f64) / -1000.0);
                    vel2.set_vyaw(0.0);

                    if default_altitude - self.z > 0.05 {
                        vel2.set_vz(default_altitude - self.z);
                    } else if default_altitude - self.z < -0.05 {
                        vel2.set_vz(default_altitude - self.z);
                    }

                    self.flight_ctrl.lock().unwrap().fly_by_velocity(&mut vel2);

                    if vision.center_x1_error.abs() < 20 && vision.center_y1_error.abs() < 20 {
                        println!("投掷");
                        is_complete_cast = true;
                        flag = 4;
                        println!("返回巡线");
                    }
                }
                4 => {
                    self.flight_ctrl.lock().unwrap().fly_to_target(&mut tar1);
                    vel2.set_vx(0.15);
                    println!("继续巡线");
                    flag = 2;
                }
                5 => {
                    let vision = self.vision_msg.lock().unwrap();
                    vel2.set_vx((vision.center_y2_error as f64) / -1000.0);
                    vel2.set_vy((vision.center_y2_error as f64) / -1000.0);
                    vel2.set_vyaw(0.0);

                    if default_altitude - self.z > 0.05 {
                        vel2.set_vz(0.03);
                    } else if default_altitude - self.z < -0.05 {
                        vel2.set_vz(-0.03);
                    }

                    self.flight_ctrl.lock().unwrap().fly_by_velocity(&mut vel2);

                    if vision.center_x2_error.abs() < 20 && vision.center_y2_error.abs() < 20 {
                        println!("降落");
                        vel2.set_vx(0.0);
                        vel2.set_vy(0.0);
                        vel2.set_vz(-0.2);
                        self.flight_ctrl
                            .lock()
                            .unwrap()
                            .fly_by_vel_duration(&mut vel2, 5.0);
                        println!("降落完成");
                        flag = 6;
                    }
                }
                _ => break,
            }
            executor.spin(SpinOptions::default().timeout(Duration::ZERO));
            sleep(Duration::from_secs(1));
        }
    }
}

// 主程序入口
fn main() {
    let quad = Quadcopter::new();
    quad.quad_init();
    rclrs::spin(quad_node.node.clone())
}