use super::state::State;

pub struct StateMachine {
    current_state: Option<Box<dyn State + Send + Sync>>,
}

#[allow(dead_code)]
impl StateMachine {
    pub fn new() -> Self {
        StateMachine { current_state: None }
    }

    // 切换状态
    pub async fn change_state(&mut self, mut new_state: Box<dyn State + Send + Sync>) {
        if let Some(mut current_state) = self.current_state.take() {
            current_state.exit().await; // 结束当前状态，等待异步操作完成
        }

        new_state.enter().await; // 进入新状态，等待异步操作完成
        self.current_state = Some(new_state);
    }

    // 更新状态
    pub async fn update(&mut self) {
        if let Some(mut state) = self.current_state.take() { // take直接转出所有权
            state.update().await; // 调用时self没有了current_state
            self.current_state = Some(state); // 归还 state
        }
    }
}
