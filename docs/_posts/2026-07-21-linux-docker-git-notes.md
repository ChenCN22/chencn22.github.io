---
title: "Linux / Docker / Git 日常踩坑速查"
layout: post
description: "磁盘占满、X11、xkill、pkill、Fcitx5、tar、Docker 清理与 GUI、git 免密与改 author、ffmpeg 抽帧、手写 SVG。"
categories: [Linux]
date: 2026-07-21
---

日常干活攒下的碎片笔记，单独成文都太短，按主题合并备查。这一篇是系统、Docker 和 Git / 媒体工具；ROS 和 Isaac Sim 的另开两篇。

## Linux / 系统

**`apt update` 报错写不了 /tmp 或缓存** —— 十有八九是 `/` 分区满了，而且常见元凶是 Docker。`df -h` 确认，`docker system df` 看占用，`docker system prune -a` 清理（注意会删掉未被容器引用的镜像）。

**磁盘满了先找大文件** —— 当前目录下按大小列出前 10 个（`-x` 不跨文件系统，不会把挂载盘算进来）：

```bash
du -axh ./ | sort -rh | head -n 10
```

**图形界面出问题** —— 先 `nvidia-smi` 确认显卡还在，再 `sudo systemctl restart gdm`。

**强杀卡死的 GUI 窗口** —— 终端敲 `xkill`，鼠标变成 X 后点谁杀谁。对卡死的 RViz / Gazebo / SSH 转发窗口特别好用。

**按命令行匹配杀进程** —— `pkill -f "match string"` 按**完整命令行**匹配（不像默认只匹配进程名），杀带一长串参数的进程 / 脚本时好用。坑：匹配串会把 `pkill`/`grep` 命令自己也匹配进去，用方括号避开（如 `patter[n]`）。

**SSH 远程打开浏览器** —— `ssh -X user@host xdg-open https://example.com`。

**切换到另一个用户** —— `su <username>`（命令行下切换用户身份，跑权限归属不同的东西时用得上）。

**原版中文输入法不好用，换 Fcitx5**

```bash
sudo apt install fcitx5 fcitx5-chinese-addons fcitx5-rime
im-config    # 选 fcitx5，注销重登生效
```

登录后命令行 `fcitx5 &` 起进程，右上角图标进 config 把 Rime 加进输入法列表，再在 Rime 里切简/繁。

**nano 里复制长行不完整** —— 长行显示被截断导致复制丢内容。`nano -l` 启动，或在 `.nanorc` 里加 `set softwrap`。

**tar 速查**

```bash
tar -xzvf file.tar.gz    # 解压 .tar.gz
tar -xjvf file.tar.bz2   # 解压 .tar.bz2
tar -xvf  file.tar       # 解压 .tar
tar -cvf  out.tar dir/   # 打包
```

## Docker

**构建基础命令**

```bash
docker build -t name:tag .              # 末尾的 . 是构建上下文
docker build -f MyDockerfile -t name:tag .
docker build --no-cache -t name:tag .   # 跳过缓存
```

**`docker kill` ≠ 清理** —— kill 只停进程，容器文件系统还在占盘。要 `docker rm -f <id>` 或 `docker ps -a` 之后 `docker container prune`。反过来也要小心：**容器一旦被 rm，容器内做过的编译/安装全部消失**，挂载卷之外的东西别当持久存储用。

**容器里跑 GUI** —— 起容器前先在宿主机执行 `xhost +local:root`，否则容器内的 GUI 程序连不上 X server。

## Git & 媒体工具

**git 免密 clone（配 SSH key）** —— 不想每次输账号密码，配一把 key：

```bash
ssh-keygen -t ed25519 -C "your_email@example.com"
cat ~/.ssh/id_ed25519.pub          # 复制输出
# 贴到 GitHub → Settings → SSH and GPG keys → New SSH key
ssh -T git@github.com              # 测试连通
```

之后用 `git@github.com:user/repo.git` 形式 clone 就不再要密码。

**改 author 重新提交，并推到新的 remote / 分支** —— 换机器或换账号后提交人信息不对时：

```bash
git config user.name  <name>
git config user.email <email>
git commit --amend --reset-author --no-edit   # 用新身份重写最后一次提交

git remote -v                                 # 如果需要换源
git remote set-url origin <link-to-.git>

git push -u origin <branch-name>
```

**ffmpeg 抽帧（比 OpenCV 快）** —— 降帧率 + 导出 JPG 序列，处理视频数据集常用：

```bash
#!/bin/bash
video_name=$1
fps=$2
base_name="${video_name%.*}"
reduced_video="${base_name}_${fps}fps.mp4"
output_dir="${base_name}_${fps}fps"

# 降帧率
ffmpeg -i "$video_name" -filter:v "fps=$fps" -c:a copy "$reduced_video"
# 抽帧为 JPG（-q:v 2 高质量，从 0 开始编号）
mkdir -p "$output_dir"
ffmpeg -i "$reduced_video" -q:v 2 -start_number 0 "${output_dir}/%05d.jpg"
```

**SVG 不用画图软件，直接写文本** —— 新建 txt 把代码敲进去，后缀改成 `.svg` 就是矢量图，浏览器/网页/Markdown 都能直接用，改尺寸颜色就是改几个数字：

```xml
<svg xmlns="http://www.w3.org/2000/svg" width="200" height="200" viewBox="0 0 200 200">
  <rect x="10" y="10" width="180" height="180" fill="none" stroke="red" stroke-width="16"/>
</svg>
```
