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