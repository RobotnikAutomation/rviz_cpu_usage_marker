# Rviz CPU Usage Marker

Shows the cpu usage on rviz

## Installation

### Requirements
* system_monitor
* visualization_msgs

## 1 rviz_cpu_usage_marker

It place a text on the frame and updates with cpu usage percent

### 1.1 Parameters

- **~output**  (`str`, default: *screen*, env: `CPU_MARKER_OUTPUT`)
   Output
-  **~debug**  (`bool`, default: *false*, env: `CPU_MARKER_DEBUG`)
   Enable/disable debug
- **~node_name** (`str`, default: *rviz_cpu_usage_marker*, env: `CPU_MARKER_NODE_NAME `)
   Node name
-  **~topic_to_subscribe**  (`str`, default: */system_monitor/diagnostics*, env: `CPU_MARKER_TOPIC_TO_SUBSCRIBE`)
   Topic to obtain the diagnostics
- **~publish_topic_name**  (`str`, default: *cpu_usage_marker*, env: `CPU_MARKER_PUBLISH_TOPIC_NAME`)
   Publish topic name
-  **~frame_id** (`str`, default: *rb1_base_map*, env: `CPU_MARKER_FRAME_ID`)
   Frame id to place marker

### 1.2 Subscribed Topics

* system_monitor/diagnostics (system_monitor/Diagnostic)
  Cpu usage data retriever

### 1.3 Published Topics

* cpu_usage_marker (visualization_msgs/Marker)
  Cpu usage percent
