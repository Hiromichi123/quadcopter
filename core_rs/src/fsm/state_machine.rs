use super::state::State;

pub struct StateMachine {
    current: Option<Box<dyn State>>,
}

#[allow(dead_code)]
impl StateMachine {
    pub fn new() -> Self {
        StateMachine { current: None }
    }

    pub fn change_state(&mut self, mut new_state: Box<dyn State>) {
        if let Some(mut current) = self.current.take() {
            current.on_exit(self);
        }
        new_state.on_enter(self);
        self.current = Some(new_state);
    }

    pub fn update(&mut self) {
        if let Some(mut state) = self.current.take() { // take直接转出所有权
            state.update(self); // 调用时self没有了current
            self.current = Some(state); // 还原 state
        }
    }
}
