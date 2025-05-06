mod quadcopter; // 无人机本体（逻辑代码）
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

    // 起飞前检查 + 找杆
    let mut flight_ctrl_node = Arc::new(Mutex::new(FlightController::new(quad_node.self_pos.clone()).expect("flight_ctrl_node: 同步飞行控制器节点创建失败")));
    stage_one(&mut flight_ctrl_node, quad_node.vision_msg.clone()); 

    // 后续飞行
    let mut flight_ctrl_node = Arc::new(AsyncMutex::new(FlightController::new(quad_node.self_pos.clone()).expect("flight_ctrl_node: 异步飞行控制器节点创建失败")));
    Runtime::new().unwrap().block_on(fly(&mut flight_ctrl_node, is_barcode_rx)); // 临时创建runtime来运行
    println!("主程序结束");
}

// 起飞前检查 + 找杆
fn stage_one(flight_controller: &mut Arc<Mutex<FlightController>>, vision_msg: Arc<Mutex<Vision>>) {
    let mut flight_ctrl = flight_controller.lock().unwrap();
    flight_ctrl.pre_flight_checks_loop().unwrap(); // 起飞前检查

    println!("预检查结束，起飞");
    let mut first_point = Target::new(0.0, 0.0, 1.5, 0.0);
    flight_ctrl.fly_to_target_sync(&mut first_point);
    println!("到达指定高度");

    // 找杆
    let mut velocity = Velocity::new(0.0, 0.0, 0.0, 0.1, 0.0, 0.0);
    while vision_msg.lock().unwrap().is_red_detected == false {
        flight_ctrl.fly_by_velocity(&mut velocity);
    }

    // 校准
    let mut velocity = Velocity::new(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    while vision_msg.lock().unwrap().later_error.abs() as f64 > 0.1 {
        velocity.set_vy(vision_msg.lock().unwrap().later_error as f64 / 100.0); 
        velocity.set_vx(vision_msg.lock().unwrap().distance as f64 / 100.0);
        flight_ctrl.fly_by_velocity(&mut velocity);
    }
}

async fn fly(flight_controller: &mut Arc<AsyncMutex<FlightController>>, is_barcode_rx: Arc<RwLock<tokio::sync::watch::Receiver<bool>>>) {
    let path = Arc::new(AsyncMutex::new(Path::new()));
    {
        path.lock().await.add_waypoint(Target::new(1.0, 0.0, 1.5, 0.0)).await;
        path.lock().await.add_waypoint(Target::new(5.0, 0.0, 1.5, 0.0)).await;
        path.lock().await.add_waypoint(Target::new(10.0, 0.0, 1.5, 0.0)).await;
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