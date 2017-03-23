echo on;
more off;
n = 10;
A = diag (sparse (1:n));
b = rand (n, 1);
[l, u, p] = ilu (A, struct ("droptol", 1.e-3));
[x, flag, relres, iter, resvec, eigest]=pcg(A,b);
flag
function y = apply_a (x)
  y = [1:size(x)(1)]' .* x;
endfunction
[x, flag, relres, iter, resvec, eigest] = pcg ("apply_a", b)
[x, flag, relres, iter, resvec, eigest] = pcg (A, b, 1.e-6, 500, l*u)
