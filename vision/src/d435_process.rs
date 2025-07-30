use anyhow::{Result, Context as AnyhowContext};
use opencv::{
    prelude::*, 
    core,
    imgproc,
};
use sensor_msgs::msg::Image;
use messages::msg::Vision;
use crate::cvchain::CvChain;
use std::sync::{Arc, Mutex};

/// 处理RGB图像数据
pub fn process_rgb_image(msg: &Image, rgb_mat: &Arc<Mutex<Mat>>) -> Result<()> {
    if msg.data.is_empty() {
        println!("接收为空");
        return Ok(());
    }

    // 转换ROSImage->RGB->BGR
    let processed_mat = CvChain::from_ros_image(msg)
        .cvt_color(imgproc::COLOR_RGB2BGR)
        .mat.clone();
    
    *rgb_mat.lock().unwrap() = processed_mat;
    
    // TODO: 在这里添加你的RGB图像处理逻辑
    
    Ok(())
}

/// 处理深度图像数据
pub fn process_depth_image(msg: &Image, depth_mat: &Arc<Mutex<Mat>>) -> Result<()> {
    if msg.data.is_empty() {
        println!("接受为空");
        return Ok(());
    }

    // 1.强制将编码设置为 "mono16"（CV_16UC1）
    let mut msg_clone = msg.clone();
    msg_clone.encoding = "mono16".to_string();

    // 2.ROSImage->DepthMat(单通道)
    let processed_mat = CvChain::from_ros_image(&msg_clone).mat.clone();
    
    *depth_mat.lock().unwrap() = processed_mat;
    
    // TODO: 在这里添加你的深度图像处理逻辑
    
    Ok(())
}