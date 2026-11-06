# raw_odometry (deprecated)

`raw_odometry` is now an alias of [`external_localization`](../external_localization/README.md)
and will be removed in a future release. It keeps reading the `raw_odometry:` parameter block
and its `odom_sub_topic` key, and logs a deprecation warning at startup.

See [Migrating from raw_odometry](../external_localization/README.md#migrating-from-raw_odometry).
