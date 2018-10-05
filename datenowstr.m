function str_ = datenowstr(varargin) 
    str_ =  datetostr(datetime('now'), varargin{:});
end