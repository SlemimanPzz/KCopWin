module KCopWin

using DataStructures
using Graphs
using GraphIO.Graph6
using Base.Threads
using Combinatorics
using OhMyThreads

export k_cop_win

"""

Given a Tuple State s = (p_0,...,p_n,t) gets the index from where it came from
`state_space_for_h`
"""
function get_index(s::NTuple{K,Int}, n::Int, k::Int) where {K}
    index = s[end] + 1

    for i in 1:(k + 1)
        index += (s[i] - 1) * ((k + 1) * n^(i - 1))
    end

    return index
end

function F(G, k)
    # Get combinations of vertices from 1:nv(G)
    vertices = collect(combinations(1:nv(G), k))

    # Create a new graph F
    F = SimpleGraph(length(vertices), 0)

    # Add edges based on neighbors
    for A in vertices
        for i in 1:length(A)
            for u in neighbors(G, A[i])
                if u ∉ A
                    B = copy(A)
                    B[i] = u
                    sort!(B)

                    # Get indices of A and B in the vertices array
                    idx_A = findfirst(x -> x == A, vertices)
                    idx_B = findfirst(x -> x == B, vertices)

                    # Add an edge between the vertex representations in F
                    if idx_A !== nothing && idx_B !== nothing
                        add_edge!(F, idx_A, idx_B)
                    end
                end
            end
        end
    end
    return F
end

"""
Generate State Space given a `n` and a `k`
given 
``S = {(p0, p1, . . . , pk, t) ∈ [n]^{k+1} × Z_{k+1}}``
"""
function state_space_for_h(n::Int, k::Int, h::SimpleDiGraph)::Array{NTuple{k + 2,UInt8},1}
    S = NTuple{k + 2,Int}[]
    for combination in Iterators.product(ntuple(_ -> 1:n, k + 1)...)
        for t in 0:k
            add_vertex!(h)
            push!(S, (combination..., t))
        end
    end
    return S
end

"""
Creates edges in helper graph H based on state relationships, where V(H) = S.
The function processes each state q ∈ S in parallel and adds edges to H using thread-safe operations.

Parameters:
- k::Int : Number of robots
- G::SimpleGraph{Int} : Graph representing the environment
- S : Collection of states
- h::SimpleDiGraph : Helper graph to store edges
"""
function create_incident_h(k::Int, G::SimpleGraph{Int}, S, h::SimpleDiGraph)
    lock = ReentrantLock()
    tforeach(collect(enumerate(S))) do (q_i, q)
        ady = get_incident_h(q, q_i, k, G, S)
        @lock lock for (q_i, s_i) in ady
            add_edge!(h, q_i, s_i)
        end
    end
end

"""
Gets all states [s] so that (q,s) is an egde in H, where V(H) = S
"""
function get_incident_h(q, q_i, k::Int, G::SimpleGraph{Int}, S)::Array{Tuple{Int,Int}}
    ady = Tuple{Int,Int}[]  # all s so that (q,s) in E(H)
    tq = q[end]
    for (s_i, s) in enumerate(S)
        ts = s[end]

        if ts != mod((tq + 1), (k + 1))
            continue
        end

        if !(q[tq + 1] in neighbors(G, s[tq + 1])) && q[tq + 1] != s[tq + 1]
            continue
        end

        if any(q[i] != s[i] for i in 1:(length(s) - 1) if i != tq + 1)
            continue
        end

        push!(ady, (q_i, s_i))
    end
    return ady
end

"""
Given a graph and a integer k, calculates if the given graph G
is winable with k cops
"""
function k_cop_win(G::SimpleGraph{Int}, k::Int)::Bool
    n = nv(G)
    if k >= n
        return true
    end

    #Create H
    h = SimpleDiGraph()
    S = state_space_for_h(n, k, h)
    create_incident_h(k, G, S, h)

    COPSWIN = Array{Bool}(undef, length(S))
    fill!(COPSWIN, false)

    COUNTER = Array{Int}(undef, length(S))
    tforeach(1:length(S)) do i
        s = S[i]
        if s[end] == 0
            COUNTER[i] = -1
        else
            COUNTER[i] = 1 + length(neighbors(G, s[1]))
        end
    end

    QUEUE = Queue{Int}()

    for (s_i, s) in enumerate(S)
        for i in 1:k
            if s[1] == s[i + 1]
                enqueue!(QUEUE, s_i)
                COPSWIN[s_i] = true
                break
            end
        end
    end

    while !isempty(QUEUE)
        s_i = dequeue!(QUEUE)

        if S[s_i][end] != 0
            for q_i in neighbors(h, s_i)
                if !COPSWIN[q_i]
                    enqueue!(QUEUE, q_i)
                    COPSWIN[q_i] = true
                end
            end
        else
            for q_i in neighbors(h, s_i)
                COUNTER[q_i] = COUNTER[q_i] - 1
                if COUNTER[q_i] == 0
                    if !COPSWIN[q_i]
                        enqueue!(QUEUE, q_i)
                        COPSWIN[q_i] = true
                    end
                end
            end
        end
    end

    cops = Vector{Int}(undef, k)
    cartesian_indices = collect(CartesianIndices(ntuple(_ -> 1:n, k)))
    res = false
    res_lock = ReentrantLock()

    @threads for c_i in 1:length(cartesian_indices)
        if res
            break
        end
        ci = cartesian_indices[c_i]
        for i in 1:k
            cops[i] = ci[i]
        end

        if all(p0 -> COPSWIN[get_index((p0, cops..., 0), n, k)], 1:n)
            @lock res_lock res = true
            break
        end
    end
    return res
end

end # module KCopWin
