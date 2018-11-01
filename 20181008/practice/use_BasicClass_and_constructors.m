addpath( '/mnt/nixbig/ownCloud/project_code/' )
addpath( '/mnt/nixbig/ownCloud/project_code/20181008/' )
addpath( '/mnt/nixbig/ownCloud/project_code/20181008/practice/' )
%%


a_bc = BasicClass(1)
b_bc = BasicClass


a_bc.Value = 5
a_bc.roundOff

b_bc.Value = 5.49999

if b_bc.roundOff > a_bc.roundOff 
    display 'There is a problem.'
else
    display 'good'
end

c_bc = BasicClass(11.05)


