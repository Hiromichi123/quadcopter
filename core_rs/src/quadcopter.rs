#![allow(dead_code)]
#![allow(unused_imports)]
use rclrs::*;
use std::sync::{Arc, Mutex};
// 自定义包
use cv_tools::msg::Vision as Vision;
// crate
use crate::flight_controller::{*};
use crate::trajectory::{*};
use crate::fsm::{*};

// 自身位姿结构体
#[derive(Default)]
pub struct SelfPos {
    pub x: f64, pub y: f64, pub z: f64, 
    pub vx: f64, pub vy: f64, pub vz: f64,
    pub ax: f64, pub ay: f64, pub az: f64,
    pub yaw: f64, pub roll: f64, pub pitch: f64,
    pub dr: f64, pub dp: f64,
}

#[derive(Clone)]
pub struct Quadcopter {
    quad_node: Arc<Node>,
    context: Arc<Context>,
    executor: Arc<Mutex<Executor>>,
    pub self_pos: Arc<Mutex<SelfPos>>, // 自身位姿
    // 订阅组
    pub vision_msg: Arc<Mutex<Vision>>,
    vision_sub: Arc<Subscription<Vision>>,
}

impl Quadcopter {
    pub fn new(async_tx: tokio::sync::watch::Sender<bool>) -> Result<Self, RclrsError> {
        let context = Arc::new(Context::default_from_env().unwrap());
        let executor = Arc::new(Mutex::new(context.create_basic_executor()));
        let quad_node: Arc<Node> = executor.lock().unwrap().create_node("quad_node")?.into();

        let self_pos = Arc::new(Mutex::new(SelfPos::default()));
        
        let vision_msg = Arc::new(Mutex::new(Vision::default()));
        let vision_msg_mut = Arc::clone(&vision_msg);
        let vision_sub = quad_node.create_subscription::<Vision, _>("vision", 
            move |msg: Vision| {
                *vision_msg_mut.lock().unwrap() = msg.clone();
                let _ = async_tx.send(msg.is_barcode_detected); // 发送异步挂起飞行
            }
        )?.into();
        println!("飞行器初始化完成");

        Ok(Quadcopter {
            quad_node, 
            context, 
            executor,
            self_pos, 
            vision_msg, vision_sub,
        })
    }

    pub fn get_self_pos(&self) -> Arc<Mutex<SelfPos>> {
        self.self_pos.clone()
    }
}
