# Note for ~/skill23_vision

This package is built from `skill23_ws/src/skill23_lookup` because `~/skill23_vision`
may be on a read-only root filesystem in some environments.

To also expose it under the vision workspace when writable:

```bash
ln -s ~/skill23_ws/src/skill23_lookup ~/skill23_vision/src/skill23_lookup
```

Then rebuild the vision workspace (or keep sourcing `skill23_ws/install`).
