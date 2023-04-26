function P = GetTwoVectorIntersection(line1_p1,line1_p2,line2_p1,line2_p2)
    a0 = line1_p1(2) - line1_p2(2);
    b0 = line1_p2(1) - line1_p1(1);
    c0 = line1_p1(1)*line1_p2(2) - line1_p2(1)*line1_p1(2);

    a1 = line2_p1(2) - line2_p2(2);
    b1 = line2_p2(1) - line2_p1(1);
    c1 = line2_p1(1)*line2_p2(2) - line2_p2(1)*line2_p1(2);
    D = a0*b1 - a1*b0;
    
    P(1) = (b0*c1 - b1*c0)/D;
    P(2) = (a1*c0 - a0*c1)/D;
end