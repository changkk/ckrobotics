function val = intersection(A,B,C,D)
    val(1) = ((A(1)*B(2)-A(2)*B(1))*(C(1)-D(1))-(A(1)-B(1))*(C(1)*D(2)-C(2)*D(1)))/((A(1)-B(1))*(C(2)-D(2))-(A(2)-B(2))*(C(1)-D(1)));
    val(2) = ((A(1)*B(2)-A(2)*B(1))*(C(2)-D(2))-(A(2)-B(2))*(C(1)*D(2)-C(2)*D(1)))/((A(1)-B(1))*(C(2)-D(2))-(A(2)-B(2))*(C(1)-D(1)));
end