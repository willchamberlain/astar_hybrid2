classdef BasicClass
   properties
      Value
   end
   methods
       function obj = BasicClass(val_)
           if nargin > 0
                obj.Value = val_ ;
           end
       end;
      function r = roundOff(obj)
         r = round([obj.Value],2);
      end
      function r = multiplyBy(obj,n)
         r = [obj.Value] * n;
      end
   end
end