create a new repository on the command line

git init
git config --global core.autocrlf false
git add *.*
git commit -m "first commit"
git remote add origin https://github.com/lamer0712/eurecar.git
git push -u origin master

push an existing repository from the command line
git status
git add *.*
git commit -m "2nd commit"   	% git commit
git push -u origin master	% git push


http://blog.sz21c.com/550
git branch mbtest1 
git push origin mbtest1 	% mbtest1 영역만 푸시
git checkout mbtest1  		% 작업 영역 변경

git checkout master
git merge mbtest1    		% master에 mbtest1 합체 (master<-- mbtest1)

git branch -d mbtest1    	% mbtest1삭제

ignore
루트 폴더에 
.gitignore
파일 생성 내용에 
*********************************************************************************************************
# 확장자가 .a인 파일 무시
*.a

# 윗 라인에서 확장자가 .a인 파일은 무시하게 했지만 lib.a는 무시하지 않음
!lib.a

# 현재 디렉토리에 있는 TODO파일은 무시하고 subdir/TODO처럼 하위디렉토리에 있는 파일은 무시하지 않음.
/TODO

# .svn/ 디렉토리에 있는 모든 파일은 무시
.svn/

# doc/notes.txt 파일은 무시하고 doc/server/arch.txt 파일은 무시하지 않음
doc/*.txt

# doc 디렉토리 아래의 모든 .txt 파일을 무시
doc/**/*.tx
*********************************************************************************************************