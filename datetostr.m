function str_ = datetostr(date_, varargin) 
    if size(varargin) > 0 
        str_ =  datestr(date_,'yyyy_mm_dd_HHMMSS');
    else
        str_ =  datestr(date_,'yyyy_mm_dd_HHMM');
    end
end