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

### 2.2 关联

```
关联操作：
git remote add origin git@github.com:maomao0532/NoteDemo.git
检查：
git remote -v
```

出现下方图示，为成功：

![image-20231220115445609](https://typora-picture-01.oss-cn-shenzhen.aliyuncs.com/image/image-20231220115445609.png)

### 2.3 将本地文件同步到github

勤commit，少push

```
1 增加修改到存储区
git add .
git status 查看仓库修改状态

2 本地提交 some description为此commit的描述
git commit -m "some description"

3 同步至github
git push

4 多个commit合并
git log 查看commit信息
git rebase -i [startpoint] [endpoint] 交互式界面对从start到end的commit操作
git rebase -i Head~2 最近两次的commit
将需要合并的commit pick改成s或squash ctrl+x退出 保存 选择路径 保存

5 向commit中添加信息
git commit --amend	ctrl+X退出即可
```

![image-20231220131215906](https://typora-picture-01.oss-cn-shenzhen.aliyuncs.com/image/image-20231220131215906.png)



### 2.4 将github同步至本地

```
1 本地无仓库时 git clone + xxx.git + 目标文件夹
git clone https://github.com/maomao0532/NoteDemo.git Notebook

2 本地存在仓库时，直接更新此仓库 
git pull
```

