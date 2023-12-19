# git使用

## 1 创建github仓库

点击new创建仓库，输入仓库名称，公开/私有，确认。

<img src="https://typora-picture-01.oss-cn-shenzhen.aliyuncs.com/image-20231220004427860.png" alt="image-20231220004427860"  />

## 2 关联仓库

### 2.1 创建本地仓库

在需要创建仓库的文件夹下，打开命令行，输入：

```
git init
```

添加.gitignore文件，过滤不用上传的文件：

```
New-Item .gitignore
用文本打开.gitignore，如下方式，可忽略文件夹内图片资源
```

![image-20231220005210363](https://typora-picture-01.oss-cn-shenzhen.aliyuncs.com/image-20231220005210363.png)

