# gnuplot -e "filename='robot1_traj.txt'" plot2d.plt
set size ratio -1
set xlabel "y"
set ylabel "x"
set xrange [*:40]
set yrange [*:*] reverse
clear
plot filename using 2:1 title "ground-truth" with lines
replot filename using 5:4 title "estimated" with lines
pause -1 "Hit any key to continue"
