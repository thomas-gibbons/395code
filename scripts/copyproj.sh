# $1 source
# $2 target

cp -r ./$1 ./$2
mv ./$2/$1.ioc ./$2/$2.ioc
sed -i -e s/$1/$2/g ./$2/.cproject
sed -i -e s/$1/$2/g ./$2/.project
sed -i -e s/$1/$2/g ./$2/.mxproject
sed -i -e s/$1/$2/g ./$2/$2.ioc
