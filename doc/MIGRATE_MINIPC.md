# miniPC 迁移快速指南

## 1. 在当前机器打包源码（不带构建产物）

```bash
cd /home/whc
tar czf engineer_whc_src.tar.gz \
  --exclude engineer_whc/build \
  --exclude engineer_whc/install \
  --exclude engineer_whc/log \
  engineer_whc
```

## 2. 复制到 miniPC 并解压

```bash
scp engineer_whc_src.tar.gz user@<miniPC_ip>:~/
ssh user@<miniPC_ip>
cd ~
tar xzf engineer_whc_src.tar.gz
```

## 3. 在 miniPC 一键安装依赖并构建

```bash
cd ~/engineer_whc
bash scripts/migrate_to_minipc.sh --ros-distro humble --ws-dir ~/engineer_whc --clean-build
```

## 4. 可选参数

```bash
bash scripts/migrate_to_minipc.sh --help
```

常用：
- `--skip-apt`：已手动装好系统依赖时使用
- `--skip-rosdep`：离线环境或你想手动装包时使用
- `--skip-bashrc`：不自动改 `~/.bashrc`
