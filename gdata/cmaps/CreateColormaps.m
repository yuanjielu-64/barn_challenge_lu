function [] = CreateColormaps(n)

names   = {'autumn', 'bone', 'cool', 'copper', 'flag', 'gray', 'hot', 'hsv', 'jet', 'lines', 'ocean', 'pink', 'prism', 'rainbow', 'spring', 'summer', 'white', 'winter'};
handles = {@autumn, @bone, @cool, @copper, @flag, @gray, @hot, @hsv, @jet, @lines, @ocean, @pink, @prism, @rainbow, @spring, @summer, @white, @winter};
for k = 1 : 1 : length(handles)
  printf('creating %s\n', names{k});
  data = handles{k}(n);
  fp = fopen([names{k}, '.cmap'], 'w');
  [a, b, c] = size(data);
  for m = 1 : 1 : a
   fprintf(fp, '%12.10f %12.10f %12.10f\n', data(m, 1), data(m, 2), data(m, 3));
  end
  fclose(fp);
 end
end
