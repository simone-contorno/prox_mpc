# prox_mpc_test_models

Test-fixture `prox_mpc::Model` plugins for the ProxMPC stack.

These are fault-injection models that exercise controller fail-safe paths the
bundled production models cannot reach.
They are registered against the same `prox_mpc::Model` base as the core models, so
[prox_mpc_controller](../prox_mpc_controller) loads them through its ordinary
`pluginlib` path during testing — no test-only seam in production code.

> **Not for production use.** This package exists only to drive tests.

## Models

| Plugin name | Class | Purpose |
| --- | --- | --- |
| `prox_mpc_test_models/NonFiniteTwist` | `NonFiniteTwistModel` | Finite linear dynamics (state `[x, y, theta]`, control `[v, omega]`) so the QP converges and reports `PROXQP_SOLVED` with a finite first control, but `toTwist()` deliberately returns a non-finite command. |

The `NonFiniteTwist` model is the only seam that reaches the controller's
non-finite-command fail-safe: a finite-mapping model cannot produce a non-finite
twist from a finite control, so this fixture is required to cover that branch.
The controller test asserts the controller brakes at the model deceleration limit
and then escalates to `nav2_core::NoValidControl` once the failure budget is
spent.

## How it is used

The package is a `<test_depend>` of `prox_mpc_controller`.
Its plugins are registered against `prox_mpc_core` via
[prox_mpc_test_models_plugins.xml](prox_mpc_test_models_plugins.xml), so the
controller's `pluginlib::ClassLoader<prox_mpc::Model>("prox_mpc_core", ...)`
discovers them by name.

## Build

```bash
colcon build --symlink-install --packages-select prox_mpc_core prox_mpc_test_models
source install/setup.bash
ros2 plugin list prox_mpc::Model   # lists prox_mpc_test_models/NonFiniteTwist
```

## License

[Apache-2.0](../LICENSE).
