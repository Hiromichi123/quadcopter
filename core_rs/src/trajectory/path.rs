use tokio::sync::Mutex;
use std::sync::Arc;
use super::target::Target;

pub struct Path {
    current_index: Arc<Mutex<usize>>,
    waypoints: Arc<Mutex<Vec<Target>>>,
}

#[allow(dead_code)]
impl Path {
    pub fn new() -> Self {
        Path {
            current_index: Arc::new(Mutex::new(0)),
            waypoints: Arc::new(Mutex::new(Vec::new())),
        }
    }

    // 添加航点(尾部)
    pub async fn add_waypoint(&self, waypoint: Target) {
        let mut waypoints = self.waypoints.lock().await;
        waypoints.push(waypoint);
    }

    // 删除航点(指定编号)
    pub async fn remove_waypoint(&mut self, erase_num: usize) -> Result<(), String> {
        let mut waypoints = self.waypoints.lock().await;

        if waypoints.is_empty() {
            return Err("path: 路径已经删空！".to_string());
        }

        if erase_num < waypoints.len() {
            waypoints.remove(erase_num);
            Ok(())
        } else {
            Err(format!("path: 非法航点号: {}", erase_num))
        }
    }

    // 获取当前航点
    pub async fn get_current_waypoint(&self) -> Option<Arc<Mutex<Target>>> {
        let index = self.current_index.lock().await;
        let waypoints = self.waypoints.lock().await;

        if *index < waypoints.len() {
            Some(Arc::new(Mutex::new(waypoints[*index].clone()))) // 返回当前航点
        } else {
            None // 当前航点不存在
        }
    }

    // 获取当前航点的编号(值)
    pub async fn get_index(&self) -> usize {
        *self.current_index.lock().await
    }

    // 下一航点
    pub async fn advance(&mut self) {
        let mut current_index = self.current_index.lock().await;
        *current_index += 1;
    }

    // 结束判断
    pub async fn done(&self) -> bool {
        let index = *self.current_index.lock().await;
        let waypoints = self.waypoints.lock();
        if index >= waypoints.await.len() {
            return true; // 所有航点都已到达
        } else {
            return false; // 还有航点未到达
        }
    }
}