import os

commands = [" git add --all"," git status", ' git commit -m"Chapter Project Update" ', " git push" ]

for i in commands: 
    os.system(i)