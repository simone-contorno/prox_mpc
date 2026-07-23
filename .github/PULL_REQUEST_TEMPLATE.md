# Description

<!-- What does this change, and why? Link any related issue with "Closes #123". -->

## Type of change

- [ ] Bug fix (non-breaking change that fixes an issue)
- [ ] New feature (non-breaking change that adds capability)
- [ ] Breaking change (existing behavior, interfaces, or parameters change)
- [ ] Documentation only
- [ ] Build, packaging, or CI

## Behavioral impact

<!--
State this explicitly even when the answer is "none". This repository treats
control behavior as the thing most worth being careful about.
-->

- Does this change any command the controller produces? <!-- yes / no -->
- Does it touch a safety path (solver-failure deceleration, `NoValidControl`
  escalation, the non-finite-command guard, the footprint veto, obstacle
  keep-out constraints, or the tracker's lifecycle teardown guard)? <!-- yes / no -->
- Does it change a default parameter value? <!-- yes / no; if yes, list them -->

## Verification

<!--
List what you actually ran and what it returned. "Should work" is not
verification. If you skipped something, say so and why.
-->

- [ ] `colcon build --symlink-install` passes
- [ ] `colcon test` passes with no new failures
- [ ] Added or updated tests covering the changed behavior
- [ ] Ran a launch or simulation reproduction (describe it below)

```text
paste the relevant command output here
```

## Checklist

- [ ] Interfaces (topics, services, actions, TF, parameters) and their docs are
      in sync with the code
- [ ] `package.xml` and `CMakeLists.txt` updated if dependencies changed, and a
      row added to `THIRD_PARTY_LICENSES.md` for any new dependency
- [ ] `CHANGELOG.rst` updated for the affected package(s)
- [ ] No secrets, keys, or credentials in the diff
- [ ] Documentation updated where behavior or configuration changed
