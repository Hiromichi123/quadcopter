#![allow(dead_code)]
#![allow(unused_imports)]
use rclrs::*;
use std::sync::{Arc, Mutex};
use anyhow::{Result, Context as AnyhowContext, anyhow};
use opencv::{
    prelude::*, 
    core,
    imgproc,
};
use sensor_msgs::msg::Image;
// 自定义包消息
use cv_tools::msg::Vision;
// 自定义cvbridge
use crate::cvbridge_rs::CvBridge;


pub struct D435Node {
    d435_node : Arc<Node>,
    context : Arc<Context>,
    executor : Arc<Mutex<Executor>>,
    // 发布组
    vision_msg : Arc<Mutex<Vision>>,
    d435_pub : Arc<Publisher<Vision>>,
    // 订阅组
    rgb_mat : Arc<Mutex<Mat>>,
    rgb_sub : Arc<Subscription<Image>>,

    depth_mat : Arc<Mutex<Mat>>,
    depth_sub : Arc<Subscription<Image>>,
}

impl D435Node {
    pub fn new() -> Result<Self> {
        let context = Arc::new(Context::default_from_env()
            .context("d435节点上下文创建失败")?
        );

        let executor = Arc::new(Mutex::new(context.create_basic_executor()));

        let d435_node : Arc<Node> = executor.lock()
            .map_err(|e| anyhow!("获取excutor锁失败: {}", e))?
            .create_node("d435_node")
            .context("无法创建d435节点")?
            .into();
        
        let vision_msg = Arc::new(Mutex::new(Vision::default()));
        let d435_pub : Arc<Publisher<Vision>> = d435_node
            .create_publisher::<Vision>("vision_d435")
            .context("无法创建d435发布器")?
            .into();

        let rgb_mat = Arc::new(Mutex::new(Mat::default()));
        let rgb_mat_mut = Arc::clone(&rgb_mat);
        let rgb_sub : Arc<Subscription<Image>> = d435_node
            .create_subscription::<Image, _>(
                "/d435/rgb",
                move |msg: Image| {
                    if msg.data.is_empty() {
                        println!("Received empty RGB image");
                        return;
                    }

                    // 1.转换Image到Mat
                    let msg_clone = msg.clone();
                    let mat_rgb = match CvBridge::imgmsg_to_cv2(&msg_clone) {
                        Ok(m) => m,
                        Err(e) => {
                            eprintln!("Failed to convert image: {}", e);
                            return;
                        }
                    };

                    // 2.转换RGB->BGR
                    let mut mat_bgr = Mat::default();
                    if let Err(e) = imgproc::cvt_color(&mat_rgb, &mut mat_bgr, imgproc::COLOR_RGB2BGR, 0) {
                        eprintln!("cvtColor failed: {}", e);
                        return;
                    }

                    // 3.存储处理后的Mat
                    *rgb_mat_mut.lock().unwrap() = mat_bgr;
                }
            )
            .context("无法创建RGB订阅器")?
            .into();

        let depth_mat = Arc::new(Mutex::new(Mat::default()));
        let depth_mat_mut = Arc::clone(&depth_mat);
        let depth_sub = d435_node
            .create_subscription::<Image, _>(
                "/d435/depth",
                move |msg: Image| {
                    if msg.data.is_empty() {
                        println!("Received empty depth image");
                        return;
                    }

                    // 1.强制将编码设置为 "mono16"（CV_16UC1）
                    let mut msg_clone = msg.clone();
                    msg_clone.encoding = "mono16".to_string();

                    // 2.使用 cv_bridge 转换为 OpenCV Mat
                    let mat = match CvBridge::imgmsg_to_cv2(&msg_clone) {
                        Ok(m) => m,
                        Err(e) => {
                            eprintln!("Failed to convert depth image: {}", e);
                            return;
                        }
                    };

                    // 3.存储 Mat
                    *depth_mat_mut.lock().unwrap() = mat;
                },
            )
            .context("无法创建depth订阅器")?
            .into();

        Ok(Self {
            d435_node,
            context,
            executor,
            vision_msg,
            d435_pub,
            rgb_mat,
            rgb_sub,
            depth_mat,
            depth_sub,
        })
    }
}
