cd ..
cd src/main/java/org/firstinspires/ftc/teamcode
for listfile in $(find InsideFullAutos.txt); do
  for file in $(cat $listfile); do
    echo $file
    cat $file | sed 's/@Disabled/\/\/@Disabled/' > /tmp/enabled
#    if [ ! -f "$file.orig" ]; then cp $file $file.orig; fi
    mv /tmp/enabled $file
  done
done