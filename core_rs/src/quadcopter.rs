use rclrs::*;
use std::{thread::sleep};
use std::time::Duration;
use std::sync::{Arc, Mutex};
// 自定义包
use vision_py::msg::Vision;
// crate
use crate::{flight_controller::FlightController, target::Target, velocity::Velocity};

// 自身位姿结构体
#[derive(Default)]
#[allow(dead_code)]
pub struct SelfPos {
    pub x: f64, pub y: f64, pub z: f64, 
    pub vx: f64, pub vy: f64, pub vz: f64,
    pub ax: f64, pub ay: f64, pub az: f64,
    pub yaw: f64, pub roll: f64, pub pitch: f64,
    pub dr: f64, pub dp: f64,
}

// 飞行器结构体
#[allow(dead_code)]
pub struct Quadcopter {
    node: Arc<Node>,
    context: Context,
    executor: Executor,
    pub self_pos: Arc<Mutex<SelfPos>>, // 自身位姿
    // 订阅组
    vision_msg: Arc<Mutex<Vision>>,
    vision_sub: Arc<Subscription<Vision>>,
}

// 飞行器内置函数
impl Quadcopter {
    // 构造函数
    pub fn new() -> Result<Self, RclrsError> {
        let context = Context::default_from_env().unwrap();
        let executor = context.create_basic_executor();
        let node = executor.create_node("quad_node")?;

        let self_pos = Arc::new(Mutex::new(SelfPos::default()));

        let vision_msg = Arc::new(Mutex::new(Vision::default()));
        let vision_msg_mut = Arc::clone(&vision_msg);
        let vision_sub = node.create_subscription::<vision_py::msg::Vision, _>("vision", 
            move |msg: Vision| {
                *vision_msg_mut.lock().unwrap() = msg;
            }
        )?;
        println!("飞行器初始化完成");

        Ok(Quadcopter {
            node, 
            context, 
            executor,
            self_pos, 
            vision_msg, vision_sub,
        })
    }

    // 主循环
    pub fn main_loop(&mut self, flight_ctrl: &mut FlightController) {
        let mut flag = 0;
        let mut first_point = Target::new(0.0, 0.0, 1.5, 0.0);
        let mut vel1 = Velocity::new(0.1, 0.0, 0.0, 0.0, 0.0, 0.0);

        loop {
            match flag {
                0 => {
                    flight_ctrl.pre_flight_checks_loop().unwrap();
                    println!("起飞");
                    flight_ctrl.fly_to_target(&mut first_point);
                    println!("到达指定高度");
                    flag = 1;
                    println!("前进");
                }
                1 => {
                    flight_ctrl.fly_by_velocity(&mut vel1);
                    if self.vision_msg.lock().unwrap().is_line_detected {
                        println!("发现直线");
                        flag = 2;
                        println!("巡线");
                    }
                }
                _ => break,
            }
            self.executor.spin(SpinOptions::default().timeout(Duration::ZERO));
            sleep(Duration::from_secs(1));
        }
    }
}
