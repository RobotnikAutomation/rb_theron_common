# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).


## [Unreleased]

### fixed
- Corrected `common.gazebo.xacro` for allowing namespaces different than `robot`
- Corrected warnings (melodic) or errors (o) about `xacro:cylinder_inertia` and `xacro:insert_block`
- Removed wheels on `rb_thero.urdf.xacro` (already on `rb_theron_base.urdf.xacro`)
- Corrected wheels inertial