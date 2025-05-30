function salt1 = calc_salt12(t,in2,uy,in4)
%CALC_SALT12
%    SALT1 = CALC_SALT12(T,IN2,UY,IN4)

%    This function was generated by the Symbolic Math Toolbox version 9.3.
%    29-Apr-2025 11:11:01

e = in4(:,3);
g = in4(:,2);
m = in4(:,1);
y_dot = in2(2,:);
t2 = t.*pi.*4.0;
t3 = cos(t2);
salt1 = reshape([-(y_dot+e.*y_dot)./(y_dot+(t3.*pi)./5.0e+3)+1.0,((uy-g.*m).*(e+1.0).*5.0e+3)./(m.*(y_dot.*5.0e+3+t3.*pi)),0.0,-e],[2,2]);
end
