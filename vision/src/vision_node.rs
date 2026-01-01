/// 原对地摄像头
#[allow(dead_code)]
#[allow(unused_imports)]
use rclrs::*;
use std::sync::{Arc, Mutex};
use anyhow::{Result, Context as AnyhowContext, anyhow};
use sensor_msgs::msg::Image;
use opencv::{
    prelude::*, 
    core,
    imgproc,
};
// 自定义msg
use messages::msg::Vision;
// 自定义mod
use crate::cvbridge_rs::CvBridge;
use crate::cvchain::CvChain;

pub struct VisionNode {
    node: Arc<Node>,
    context: Arc<Context>,
    pub executor: Arc<Mutex<Executor>>,
    // 发布组
    vision_msg: Arc<Mutex<Vision>>,
    vision_pub: Arc<Publisher<Vision>>,
    // 订阅组
    frame_sub: Arc<Subscription<Image>>,
}

impl VisionNode {
    pub fn new() -> Result<Self> {
        let context = Arc::new(Context::default_from_env()
            .context("vision节点上下文创建失败")?
        );

        let executor = Arc::new(Mutex::new(context.create_basic_executor()));

        let node: Arc<Node> = executor.lock()
            .map_err(|e| anyhow!("获取executor锁失败: {}", e))?
            .create_node("vision_node")
            .context("无法创建vision节点")?
            .into();

        let vision_msg = Arc::new(Mutex::new(Vision::default()));
        let vision_pub : Arc<Publisher<Vision>> = node
            .create_publisher::<Vision>("vision")
            .context("无法创建vision发布器")?
            .into();

        let frame_sub : Arc<Subscription<Image>> = node
            .create_subscription::<Image, _>(
                "/camera/ground",
                move |msg: Image| {
                    if msg.data.is_empty() {
                        println!("Received empty image");
                        return;
                    }

                    // Image->Mat
                    let msg_clone = msg.clone();
                    let mat_frame = match CvBridge::imgmsg_to_cv2(&msg_clone) {
                        Ok(m) => m,
                        Err(e) => {
                            println!("Failed to convert Image to Mat: {}", e);
                            return;
                        }
                    };
                    
                    // 处理接收到的消息
                    let _ = CvChain::from_ros_image(&msg_clone)
                        .show("origin mat");
                    
                }
            )
            .context("无法创建visoion订阅器")?
            .into();

        Ok(Self {
            node,
            context,
            executor,
            vision_msg,
            vision_pub,
            frame_sub,
        })
    }
}