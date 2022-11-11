function [posOut] = posconvert(trID,posIn)
    transl = [1 0 5;0 1 5;0 0 1];
    rot = rotz(180);
    M2C = transl*rot;
    in = posIn';
    sz = size(in);
    k = ones(1,sz(2));
    inhomog = [in;k];
    if trID == "M2C"
        outhomog = M2C * inhomog;
    else
        outhomog = M2C \ inhomog;
    end
    posOut = outhomog(1:2,:)';
end