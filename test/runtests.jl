using KCopWin
using Test
using Graphs
using GraphIO.Graph6


@testset "Petersen" begin
    @test k_cop_win(smallgraph(:petersen), 3)
end

@testset "Cicles k = 1" begin
    k = 1
    @test all(k_cop_win(cycle_graph(i), k) for i in 1:3)
    @test all(!k_cop_win(cycle_graph(i), k) for i in 4:10)
end

@testset "Cicles k = 2" begin
    k = 2
    @test all(k_cop_win(cycle_graph(i), k) for i in 1:10)
end

@testset "Chordal 4" begin
    graphs = loadgraphs("../graphs/chordal4.g6", Graph6Format())
    @test all((k_cop_win(g.second, 1) for g in graphs))
end

"""
Given the dimension of a cube return its cop number.
"""
function get_cube_cop_num(dim::Int):Int
    return div(dim + 1, 2, RoundUp)
end

@testset "Cube" begin
    @test k_cop_win(Graphs.grid([2,2]), get_cube_cop_num(2))
    @test k_cop_win(Graphs.grid([2,2,2]), get_cube_cop_num(3)) skip = true # May take minutes
    @test k_cop_win(Graphs.grid([2,2,2,2]), get_cube_cop_num(4)) skip = true # May take hours
end