#begin_lua

function approach_cost(v0, v1, v2)
	s0 = tostring(v0)
	s1 = tostring(v1)
	s2 = tostring(v2)
	if s0 == "d3_414b2" then
		if s1 == "d3_414b2" then
			if s2 == "l3_400" then
				return 1 -- combo not sampled.
			end
			if s2 == "l3_414b" then
				return 1 -- combo not sampled.
			end
			if s2 == "l3_418" then
				return 1 -- combo not sampled.
			end
			if s2 == "l3_432" then
				return 1 -- combo not sampled.
			end
		end
		if s1 == "d3_418" then
			if s2 == "l3_400" then
				return 23
			end
			if s2 == "l3_414b" then
				return 1 -- combo not sampled.
			end
			if s2 == "l3_418" then
				return 1 -- combo not sampled.
			end
			if s2 == "l3_432" then
				return 1 -- combo not sampled.
			end
		end
		if s1 == "d3_432" then
			if s2 == "l3_400" then
				return 1 -- combo not sampled.
			end
			if s2 == "l3_414b" then
				return 1 -- combo not sampled.
			end
			if s2 == "l3_418" then
				return 1 -- combo not sampled.
			end
			if s2 == "l3_432" then
				return 1 -- combo not sampled.
			end
		end
		if s1 == "none" then
			if s2 == "l3_400" then
				return 1 -- combo not sampled.
			end
			if s2 == "l3_414b" then
				return 8
			end
			if s2 == "l3_418" then
				return 1 -- combo not sampled.
			end
			if s2 == "l3_432" then
				return 1 -- combo not sampled.
			end
		end
	end
	if s0 == "d3_418" then
		if s1 == "d3_414b2" then
			if s2 == "l3_400" then
				return 1 -- combo not sampled.
			end
			if s2 == "l3_414b" then
				return 1 -- combo not sampled.
			end
			if s2 == "l3_418" then
				return 1 -- combo not sampled.
			end
			if s2 == "l3_432" then
				return 1 -- combo not sampled.
			end
		end
		if s1 == "d3_418" then
			if s2 == "l3_400" then
				return 1 -- combo not sampled.
			end
			if s2 == "l3_414b" then
				return 1 -- combo not sampled.
			end
			if s2 == "l3_418" then
				return 4
			end
			if s2 == "l3_432" then
				return 1 -- combo not sampled.
			end
		end
		if s1 == "d3_432" then
			if s2 == "l3_400" then
				return 35
			end
			if s2 == "l3_414b" then
				return 1 -- combo not sampled.
			end
			if s2 == "l3_418" then
				return 1 -- combo not sampled.
			end
			if s2 == "l3_432" then
				return 1 -- combo not sampled.
			end
		end
		if s1 == "none" then
			if s2 == "l3_400" then
				return 1 -- combo not sampled.
			end
			if s2 == "l3_414b" then
				return 1 -- combo not sampled.
			end
			if s2 == "l3_418" then
				return 1 -- combo not sampled.
			end
			if s2 == "l3_432" then
				return 1 -- combo not sampled.
			end
		end
	end
	if s0 == "d3_432" then
		if s1 == "d3_414b2" then
			if s2 == "l3_400" then
				return 15
			end
			if s2 == "l3_414b" then
				return 1 -- combo not sampled.
			end
			if s2 == "l3_418" then
				return 1 -- combo not sampled.
			end
			if s2 == "l3_432" then
				return 1 -- combo not sampled.
			end
		end
		if s1 == "d3_418" then
			if s2 == "l3_400" then
				return 1 -- combo not sampled.
			end
			if s2 == "l3_414b" then
				return 1 -- combo not sampled.
			end
			if s2 == "l3_418" then
				return 1 -- combo not sampled.
			end
			if s2 == "l3_432" then
				return 1 -- combo not sampled.
			end
		end
		if s1 == "d3_432" then
			if s2 == "l3_400" then
				return 1 -- combo not sampled.
			end
			if s2 == "l3_414b" then
				return 1 -- combo not sampled.
			end
			if s2 == "l3_418" then
				return 1 -- combo not sampled.
			end
			if s2 == "l3_432" then
				return 4
			end
		end
		if s1 == "none" then
			if s2 == "l3_400" then
				return 1 -- combo not sampled.
			end
			if s2 == "l3_414b" then
				return 1 -- combo not sampled.
			end
			if s2 == "l3_418" then
				return 1 -- combo not sampled.
			end
			if s2 == "l3_432" then
				return 1 -- combo not sampled.
			end
		end
	end
	return 1 -- return 1 for any action not seen previously.
end

function askperson_cost(v0)
	s0 = tostring(v0)
	return 1 -- return 1 for any action not seen previously.
end

function gothrough_cost(v0)
	s0 = tostring(v0)
	if s0 == "d3_414b2" then
		return 5
	end
	if s0 == "d3_418" then
		return 4
	end
	if s0 == "d3_432" then
		return 6
	end
	return 1 -- return 1 for any action not seen previously.
end

function opendoor_cost(v0, v1)
	s0 = tostring(v0)
	s1 = tostring(v1)
	if s0 == "d3_414b2" then
		if s1 == "l3_400" then
			return 1 -- combo not sampled.
		end
		if s1 == "l3_414b" then
			return 1
		end
	end
	if s0 == "d3_418" then
		if s1 == "l3_400" then
			return 1
		end
		if s1 == "l3_414b" then
			return 1 -- combo not sampled.
		end
	end
	if s0 == "d3_432" then
		if s1 == "l3_400" then
			return 4
		end
		if s1 == "l3_414b" then
			return 1 -- combo not sampled.
		end
	end
	return 1 -- return 1 for any action not seen previously.
end

function remind_cost()
	return 19
end

function searchroom_cost(v0)
	s0 = tostring(v0)
	if s0 == "l3_400" then
		return 21
	end
	if s0 == "l3_414b" then
		return 15
	end
	return 1 -- return 1 for any action not seen previously.
end

#end_lua.
