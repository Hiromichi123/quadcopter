mod quadcopter; // 无人机本体（逻辑代码）
mod flight_controller; // 飞行控制器
mod fsm; // 有限状态机
mod trajectory; // 轨迹库
use std::sync::{Arc, RwLock};
use tokio::sync::Mutex as AsyncMutex;
use tokio::runtime::Runtime;

use crate::quadcopter::Quadcopter;
use crate::flight_controller::FlightController;
use crate::trajectory::{Target, Path};

// 主程序入口
fn main() {
    let (is_square_tx, is_square_rx) = tokio::sync::watch::channel(false);
    let is_square_rx = Arc::new(RwLock::new(is_square_rx));

    let quad_node = Quadcopter::new(is_square_tx).expect("quad_node:飞行器节点创建失败");
    let mut flight_ctrl_node = Arc::new(AsyncMutex::new(FlightController::new(quad_node.self_pos.clone()).expect("flight_ctrl_node:飞行控制器节点创建失败")));
    Runtime::new().unwrap().block_on(fly(&mut flight_ctrl_node, is_square_rx)); // 临时创建runtime来运行
    println!("主程序结束");
}

async fn fly(flight_controller: &mut Arc<AsyncMutex<FlightController>>, is_square_rx: Arc<RwLock<tokio::sync::watch::Receiver<bool>>>) {
    let path = Arc::new(AsyncMutex::new(Path::new()));
    {
        path.lock().await.add_waypoint(Target::new(1.0, 0.0, 1.5, 0.0)).await;
        path.lock().await.add_waypoint(Target::new(5.0, 0.0, 1.5, 0.0)).await;
        path.lock().await.add_waypoint(Target::new(10.0, 0.0, 1.5, 0.0)).await;
    }

    // 路径执行前
    {
        let mut flight_ctrl = flight_controller.lock().await;
        flight_ctrl.pre_flight_checks_loop().unwrap(); // 起飞前检查

        println!("预检查结束，起飞");
        let mut first_point = Target::new(0.0, 0.0, 1.5, 0.0);
        flight_ctrl.fly_to_target_sync(&mut first_point);
        println!("到达指定高度");
    }

    // 执行异步路径飞行
    let flight_ctrl_clone = Arc::clone(flight_controller);
    let path_clone = Arc::clone(&path);
    let task_handle = tokio::spawn(async move {
        let mut flight_ctrl = flight_ctrl_clone.lock().await;
        flight_ctrl.fly_by_path(path_clone, is_square_rx.clone()).await;
    });

    task_handle.await.unwrap(); // 等待路径完成
}