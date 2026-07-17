# MoveIt Config Variants

This directory stores version-specific MoveIt configuration snapshots.

- `V1.0/`: current production MoveIt configuration copied from `config/`.
- `V1.1/`: reserved for the regenerated V1.1 SRDF, collision matrix, and named states.

Keep the root `config/` files in place until the launch files are switched to load
variant-specific configuration.
