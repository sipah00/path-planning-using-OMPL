#set terminal postscript eps color enhanced 20
#set output "out.eps"
set xlabel "x"
set ylabel "y"
set xrange [0:22]
set yrange [0:22]
set key outside
set key top right
set size square
plot "obstacle.dat" using 1:2 with filledcurves lt rgb "#ff0033" fill solid 0.5 notitle,\
"start.dat" using 1:2 with points pt 7 ps 1.5 lt rgb "#ff9900" title "Start",\
"goal.dat" using 1:2 with points pt 7 ps 1.5 lt rgb "#15BB15" title "Goal",\
"edges.dat" using 1:2 with lines lt 1 lc rgb "#728470" lw 0.5 title "edges",\
"path.dat" using 1:2 with lines lt 1 lc rgb "#191970" lw 2 title "Path",\
"path0.dat" using 1:2 with lines lt 1 lc rgb "#ff4500" lw 2 title "Path0"
