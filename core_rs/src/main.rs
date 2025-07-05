#![allow(dead_code)]
#![allow(unused_imports)]
mod quadcopter; // 无人机
mod flight_controller; // 飞行控制器
mod fsm; // 有限状态机
mod trajectory; // 轨迹库
use std::sync::{Arc, RwLock, Mutex};
use tokio::sync::Mutex as AsyncMutex;
use tokio::runtime::Runtime;

use crate::quadcopter::Quadcopter;
use crate::flight_controller::FlightController;
use crate::trajectory::{Target, Velocity, Path};
use cv_tools::msg::Vision as Vision;

// 主程序入口
fn main() {
    let (is_barcode_tx, is_barcode_rx) = tokio::sync::watch::channel(false);
    let is_barcode_rx = Arc::new(RwLock::new(is_barcode_rx));

    let quad_node = Quadcopter::new(is_barcode_tx).expect("quad_node:飞行器节点创建失败");

    // 起飞+找杆
    let mut flight_ctrl_node = Arc::new(Mutex::new(FlightController::new(quad_node.self_pos.clone()).expect("flight_ctrl_node: 同步飞行控制器节点创建失败")));
    Runtime::new().unwrap().block_on(stage_one(&mut flight_ctrl_node, quad_node.vision_msg.clone())); 

    // 记录当前位姿
    let self_pos = quad_node.get_self_pos();
    // 根据yaw和distance计算杆的坐标
    let x = self_pos.lock().unwrap().x + quad_node.vision_msg.lock().unwrap().distance * self_pos.lock().unwrap().yaw.cos();
    let y = self_pos.lock().unwrap().y + quad_node.vision_msg.lock().unwrap().distance * self_pos.lock().unwrap().yaw.sin();
    print!("杆坐标: ({}, {})\n", x, y);
    // 二维码点
    let mut target = Target::new(x, y+0.5, 1.5, 1.57);
    let _ = flight_ctrl_node.lock().unwrap().fly_to_target_sync(&mut target);
    target.set_z(1.0);
    let _ = flight_ctrl_node.lock().unwrap().fly_to_target_sync(&mut target);
    println!("二维码点到达");

    // 速度飞行
    let mut velocity = Velocity::new(0.0, 0.2, 0.0, 0.0, 0.0, 0.0);
    let _ = Runtime::new().unwrap().block_on(flight_ctrl_node.lock().unwrap().fly_by_vel_duration(&mut velocity, 10.0, is_barcode_rx));
    
    // 后续飞行
    //Runtime::new().unwrap().block_on(fly(&mut flight_ctrl_node, is_barcode_rx)); // 临时创建runtime来运行
    println!("主程序结束");
}

// 起飞 + 找杆
async fn stage_one(flight_controller: &mut Arc<Mutex<FlightController>>, vision_msg: Arc<Mutex<Vision>>) {
    let mut flight_ctrl = flight_controller.lock().unwrap();
    match flight_ctrl.pre_flight_checks_loop().await {
        Ok(_) => println!("预检查通过, 起飞"),
        Err(e) => println!("预检查失败: {}", e),
    };

    let mut first_point = Target::new(0.0, 0.0, 1.0, 0.0);
    match flight_ctrl.fly_to_target_sync(&mut first_point) {
        Ok(_) => println!("到达定高"),
        Err(e) => println!("定高失败: {}", e),
    };

    // 找杆
    let mut velocity = Velocity::new(0.0, 0.0, 0.0, 0.1, 0.0, 0.0);
    while vision_msg.lock().unwrap().is_red_detected == false {
        let _ = flight_ctrl.fly_by_velocity(&mut velocity);
    }
    println!("杆found, 开始校准");

    // 校准
    velocity.set_vyaw(0.0);
    while vision_msg.lock().unwrap().later_error.abs() > 50 {
        velocity.set_vy(vision_msg.lock().unwrap().later_error as f64 / 100.0); 
        let _ = flight_ctrl.fly_by_velocity(&mut velocity);
    }
    println!("校准完成");
}

// 异步飞行, 未使用
async fn fly(flight_controller: &mut Arc<AsyncMutex<FlightController>>, is_barcode_rx: Arc<RwLock<tokio::sync::watch::Receiver<bool>>>) {
    let path = Arc::new(AsyncMutex::new(Path::new()));
    {
        let targets = vec![
            Target::new(1.0, 0.0, 1.5, 0.0),
            Target::new(5.0, 0.0, 1.5, 0.0),
            Target::new(10.0, 0.0, 1.5, 0.0),
        ];

        let mut path_locked = path.lock().await;
        path_locked.add_waypoints(targets).await;
    }

    // 执行异步路径飞行
    let flight_ctrl_clone = Arc::clone(flight_controller);
    let path_clone = Arc::clone(&path);
    let task_handle = tokio::spawn(async move {
        let mut flight_ctrl = flight_ctrl_clone.lock().await;
        flight_ctrl.fly_by_path(path_clone, is_barcode_rx.clone()).await;
    });

    task_handle.await.unwrap(); // 等待路径完成
}