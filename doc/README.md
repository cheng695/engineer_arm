# engineer_whc

## 快速入口

- 实车启动说明：[README_real_robot.md](/home/rcia/Desktop/engineer_whc/README_real_robot.md)
- miniPC 迁移说明：[MIGRATE_MINIPC.md](/home/rcia/Desktop/engineer_whc/MIGRATE_MINIPC.md)

## 常用命令

实车主系统启动：

```bash
cd ~/Desktop/engineer_whc
bash scripts/start_real_robot.sh --bitrate 1000000
```

第二个终端启动 commander：

```bash
cd ~/Desktop/engineer_whc
bash scripts/start_commander.sh
```

## 脚本说明

- [scripts/start_real_robot.sh](/home/rcia/Desktop/engineer_whc/scripts/start_real_robot.sh)：配置 CAN 并启动实车主系统
- [scripts/start_commander.sh](/home/rcia/Desktop/engineer_whc/scripts/start_commander.sh)：启动 commander 节点
- [scripts/migrate_to_minipc.sh](/home/rcia/Desktop/engineer_whc/scripts/migrate_to_minipc.sh)：在 miniPC 上安装依赖并构建工作区
