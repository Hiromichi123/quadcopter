#![allow(dead_code)]
#![allow(unused_variables)]
#![allow(unused_imports)]
mod cvbridge_rs;
mod cvchain;
mod d435_node;
mod vision_node;
use anyhow::{Result, Context};

fn main() -> Result<()> {
    let d435_node = d435_node::D435Node::new()
        .context("D435节点创建失败")?;
    
    let vision_node = vision_node::VisionNode::new()
        .context("Vision节点创建失败")?;
    
    Ok(())
}