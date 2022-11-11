function [oriOut] = oriconvert(oriIn)
    oriOut = pi - oriIn;
    if oriIn > 0
        oriOut = -pi + oriIn;
    end
    if oriIn < 0
        oriOut = pi + oriIn;
    end
end