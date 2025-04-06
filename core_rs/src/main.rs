mod quadcopter;
mod flight_controller;
mod target;
mod velocity;

// 主程序入口
fn main() {
    let mut quad_node = quadcopter::Quadcopter::new().expect("Failed to create quad_node");
    let mut flight_ctrl_node = flight_controller::FlightController::new(quad_node.self_pos.clone()).expect("Failed to create flight_ctrl_node");
    quad_node.main_loop(&mut flight_ctrl_node);
    println!("主程序结束");
}