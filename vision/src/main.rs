#![allow(dead_code)]
#![allow(unused_variables)]
#![allow(unused_imports)]
mod cvbridge_rs;
mod cvchain;
mod d435_node;
mod vision_node;
use rclrs::*;
use std::sync::{Arc, Mutex};
use std::time::Duration;
use d435_node::D435Node;
use vision_node::VisionNode;
use anyhow::{Result, Context};

fn main() -> Result<()> {
    let d435_node = d435_node::D435Node::new()
        .context("D435节点创建失败")?;

        let vision_node = vision_node::VisionNode::new()
        .context("Vision节点创建失败")?;

        let d435_thread = std::thread::spawn(move || {
        d435_node.executor.lock()
                .unwrap()
                .spin(SpinOptions::default().timeout(Duration::from_millis(20)));
    });
    
    let vision_thread = std::thread::spawn(move || {
        vision_node.executor.lock()
            .unwrap()
            .spin(SpinOptions::default().timeout(Duration::from_millis(20)));
    });

    let _ = d435_thread.join();
    //let _ = vision_thread.join();

    Ok(())
}