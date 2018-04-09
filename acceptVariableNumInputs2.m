function acceptVariableNumInputs2(num1, num2, num3, varargin)
    display(fprintf('size(varargin)=%d',size(varargin) ))
    display(fprintf('Number of input arguments: %d', nargin ))
    display(fprintf( 'Number of varargin arguments: %d', (size(varargin) - nargin) ))
    celldisp(varargin)
end