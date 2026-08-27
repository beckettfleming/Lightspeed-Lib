#include "lightspeed/driver/demo_macros.hpp"

namespace lightspeed::driver::demo {

ButtonMacro makeDemoArmCycleMacro(subsystem::demo::ExampleArm& exampleArm) {
    return ButtonMacro{
        .name = "Demo: arm cycle",
        .steps =
            {
                MacroStep{
                    .action = [&exampleArm] { exampleArm.moveToPreset("HIGH"); },
                    .isDone = [&exampleArm] { return exampleArm.getState() == subsystem::demo::ExampleArmState::holding; },
                },
                MacroStep{
                    .action = [&exampleArm] { exampleArm.moveToPreset("LOW"); },
                    // Default isDone (always true): fire-and-forget the
                    // final step, no need to wait for it before the macro
                    // itself reports done.
                },
            },
    };
}

}  // namespace lightspeed::driver::demo
