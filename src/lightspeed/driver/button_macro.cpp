#include "lightspeed/driver/button_macro.hpp"

namespace lightspeed::driver {

void ButtonMacroRunner::trigger(const ButtonMacro& macro) {
    activeMacro_ = &macro;
    currentStepIndex_ = 0;
    currentStepActionRun_ = false;
}

void ButtonMacroRunner::update() {
    if (activeMacro_ == nullptr) {
        return;
    }
    if (currentStepIndex_ >= activeMacro_->steps.size()) {
        activeMacro_ = nullptr;
        return;
    }

    const MacroStep& step = activeMacro_->steps[currentStepIndex_];
    if (!currentStepActionRun_) {
        if (step.action) {
            step.action();
        }
        currentStepActionRun_ = true;
    }

    if (step.isDone && step.isDone()) {
        ++currentStepIndex_;
        currentStepActionRun_ = false;
        if (currentStepIndex_ >= activeMacro_->steps.size()) {
            activeMacro_ = nullptr;
        }
    }
}

bool ButtonMacroRunner::isRunning() const {
    return activeMacro_ != nullptr;
}

}  // namespace lightspeed::driver
