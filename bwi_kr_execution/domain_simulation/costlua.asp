#begin_lua

function approach_cost(v0)
	s0 = tostring(v0)
	return 1 -- return 1 for any action not seen previously.
end

function askperson_cost(v0, v1)
	s0 = tostring(v0)
	s1 = tostring(v1)
	return 1 -- return 1 for any action not seen previously.
end

function gothrough_cost(v0)
	s0 = tostring(v0)
	return 1 -- return 1 for any action not seen previously.
end

function opendoor_cost(v0)
	s0 = tostring(v0)
	return 1 -- return 1 for any action not seen previously.
end

function remind_cost(v0, v1, v2)
	s0 = tostring(v0)
	s1 = tostring(v1)
	s2 = tostring(v2)
	return 1 -- return 1 for any action not seen previously.
end

function searchroom_cost(v0, v1)
	s0 = tostring(v0)
	s1 = tostring(v1)
	return 1 -- return 1 for any action not seen previously.
end

#end_lua.
