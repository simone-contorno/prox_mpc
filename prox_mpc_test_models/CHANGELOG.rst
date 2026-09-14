^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package prox_mpc_test_models
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.0 (2026-09-13)
------------------
* Added fault and mapping fixtures: asymmetric control bounds, a permuted planar
  mapping, and a non-finite-other-axes twist model.
* Contributors: Simone Contorno

1.0.0 (2026-07-28)
------------------
* Initial release: fault-injection ``prox_mpc::Model`` plugin (``NonFiniteTwist``)
  used by the controller test suite to exercise fail-safe branches through the
  real pluginlib load path.
* Contributors: Simone Contorno
