import math
from enum import Enum, unique


# from amaranth.lib.Memory import
from amaranth.lib import *


from amaranth import Elaboratable, Signal, Module, Const, Array, signed, ResetSignal
# from amaranth import Display
from amaranth.build import Platform
from amaranth.sim import Simulator,Delay, _pycoro
from amaranth.asserts import Assert, Assume, Cover 
# from migen import Display


@unique
class states(Enum):
    preprocessing   =0
    neighbours      =1
    # validunblock    =2
    End             =3
    non              =4
    # gcal            =5
    north             =6
    print           =7
    isdest          =8
    south           =9
    east            =10
    west            =11
    Found           =12
    cantfind        =13



class abc(Elaboratable):
    def __init__(self):
        self.srci=Signal(4) #srouce point's index i
        self.srcj=Signal(4) #srouce point's index j
        self.desti=Signal(4) #destination points's indesx i
        self.destj=Signal(4) #destination points's indesx j
        self.found=Signal() # to tell desination is reached
        self.legal=Signal() # it legal to start the path-tracing
        self.i=Signal(4) #for storing the index value i after finding it
        self.j=Signal(4) #for storing the index value j after finding it
        self.temi=Signal(4)
        self.temj=Signal(4) # temparory values to store src index j
        self.nn=Signal(3)# counting no of neighbours
        self.p=Signal()
        self.start=Signal()

        self.gr=Array(Array(Signal() for _ in range(4))for __ in range(4)) # storing values of matrix


    def ports(self):
        return [self.desti,self.destj,self.srci,self.srcj,self.found,self.i,self.j]

    def elaborate(self,platform:Platform):
        m=Module()

        # assigning the value sto grid
        m.d.sync+=[
            self.gr[0][0].eq(1),
            self.gr[0][1].eq(1),
            self.gr[0][2].eq(1),
            self.gr[0][3].eq(1),
            self.gr[1][0].eq(1),
            self.gr[1][1].eq(1),
            self.gr[1][2].eq(1),
            self.gr[1][3].eq(0),
            self.gr[2][0].eq(1),
            self.gr[2][1].eq(0),
            self.gr[2][2].eq(0),
            self.gr[2][3].eq(0),
            self.gr[3][0].eq(1),
            self.gr[3][1].eq(1),
            self.gr[3][2].eq(0),
            self.gr[3][3].eq(1),
        ]
        # # # # #
        # 1 1 1 1
        # 1 1 1 0
        # 1 0 0 0
        # 1 1 0 1
        self.elaborateStateMachine(m,platform)
        return m

    def elaborateStateMachine(self,m:Module,platform:Platform):
        self.st=Signal(states)
        self.var=Signal(states)
        #to store state changes
        self.g=Signal(32) 
        # to store the changing value of g
        self.xi=Signal(4)
        self.xj=Signal(4) 
        # storing the changing values of i,j
        self.y1=Signal(4)
        self.y2=Signal(4)

        with m.FSM() as fsm:
            with m.State(states.preprocessing):
                    with m.If(self.start==1): # while we have the start signal as 1
                        m.d.sync+=self.st.eq(states.preprocessing),
                        #assigning y1 to 0 and y2 to 4
                        m.d.comb+=[
                                self.y1.eq(0),
                                self.y2.eq(4)
                        ]
                        with m.If(self.srci>=self.y1):
                            with m.If(self.srcj<self.y2):
                                with m.If(self.srcj>=self.y1):
                                    with m.If( self.srcj<self.y2):
                                        with m.If(self.gr[self.srci][self.srcj]==1):# 1-unblocked and 0- is_blocked
                                            m.d.comb+=self.p.eq(1)
                        # if the srouce is valid and unblocked
                        with m.If (self.p==1):
                            m.d.sync+=self.legal.eq(1) # showing that it is legal to do path_finding
                            m.d.comb+=self.i.eq(self.srci)
                            m.d.comb+=self.j.eq(self.srcj)
                            m.d.sync+=self.temi.eq(self.srci) # assigning the values to i,j and temi,temj
                            m.d.sync+=self.temj.eq(self.srcj)
                            m.next=states.neighbours
                        # if srouce is invalid or blocked to goes to end stage
                        with m.Else():
                            m.next=states.End

            with m.State(states.neighbours):
                m.d.sync+=self.st.eq(states.neighbours)
                # just assign the value to g for calculations
                m.d.sync+=self.g.eq(9999)
                m.d.comb+=self.nn.eq(0)
                m.next=states.north

            with m.State(states.north):

                m.d.sync+=self.st.eq(states.north)

                ni=Signal(4)
                nj=Signal(4) # calculating the position of the lower cell

                gn=Signal(32) # storing the values the calculated distance form the destination to lower


                m.d.comb+=[
                    ni.eq(self.temi+1),
                    nj.eq(self.temj),
                ] # calculating position of lower cell

                m.d.comb+=[
                    self.y1.eq(self.xi),
                    self.y2.eq(self.xj)
                ]# storing the current values in y1,y2
                #checking if lower cell is valid and unblocked
                with m.If(ni>=0):
                    with m.If(ni<4):
                        with m.If(nj>=0):
                            with m.If(nj<4):
                                with m.If(self.gr[ni][nj]==1):
                                    m.d.comb+=[gn.eq((self.desti*self.desti)+(self.destj*self.destj)+(ni*ni)+(nj*nj)-(2*self.desti*ni)-(2*self.destj*nj)),
                                        # self.nn.eq(self.nn+1)
                                    ]
                                    #calculating the distance form destination
                                    with m.If(self.g>gn):
                                        m.d.sync+=[
                                        self.g.eq(gn),
                                        self.xi.eq(ni),
                                        self.xj.eq(nj)
                                        ]
                m.next=states.south

            with m.State(states.south):
                m.d.sync+=self.st.eq(states.south)
                si=Signal(4)
                sj=Signal(4) # for storing the values of upper cell
                gs=Signal(32) # storing the distance from destination to upper cell
                m.d.comb+=[
                    si.eq(self.temi-1),
                    sj.eq(self.temj),
                    gs.eq(0),
                    self.y1.eq(self.xi),
                    self.y2.eq(self.xj)
                ]
                # Checking if upper cell is valid and unblocked
                with m.If(si>=0):
                        with m.If(si<4):
                            with m.If(sj>=0):
                                with m.If(sj<4):
                                    with m.If(self.gr[si][sj]==1):
                                        #calculating the  distances
                                        m.d.comb+=[gs.eq((self.desti*self.desti)+(self.destj*self.destj)+(si*si)+(sj*sj)-(2*self.desti*si)-(2*self.destj*sj)),
                                                #    self.nn.eq(self.nn+1)
                                                   ]
                                        with m.If(self.g>gs):
                                            m.d.sync+=[
                                            self.g.eq(gs),
                                            self.xi.eq(si),
                                            self.xj.eq(sj)
                                            ]
                # to next state
                m.next=states.east

            with m.State(states.east):
                m.d.sync+=self.st.eq(states.east)
                ei=Signal(4)
                ej=Signal(4) # storing the values of front cell
                ge=Signal(32) # storing the distance from destination to front cell
                m.d.comb+=[
                    ei.eq(self.temi),
                    ej.eq(self.temj+1),
                    ge.eq(0)
                ]
                with m.If(ei>=0):
                        with m.If(ei<4):
                            with m.If(ej>=0):
                                with m.If(ej<4):
                                    with m.If(self.gr[ei][ej]==1):
                                        m.d.comb+=[ge.eq((self.desti*self.desti)+(self.destj*self.destj)+(ei*ei)+(ej*ej)-(2*self.desti*ei)-(2*self.destj*ej)),
                                                #    self.nn.eq(self.nn+1)
                                                   ]
                                        with m.If(self.g>ge):
                                            m.d.sync+=[
                                                self.xi.eq(ei),
                                                self.xj.eq(ej),
                                                self.g.eq(ge)
                                                ]
                m.next=states.west

            with m.State(states.west):
                m.d.sync+=self.st.eq(states.west)

                wi=Signal(4)
                wj=Signal(4)
                gw=Signal(32)
                w=Signal()
                m.d.comb+=[
                    wi.eq(self.temi-1),
                    wj.eq(self.temj),
                    gw.eq(0),
                    w.eq(0)
                ]
                with m.If(wi>=0):
                        with m.If(wi<4):
                            with m.If(wj>=0):
                                with m.If(wj<4):
                                    with m.If(self.gr[wi][wj]==1):
                                        m.d.comb+=[gw.eq((self.desti*self.desti)+(self.destj*self.destj)+(wi*wi)+(wj*wj)-(2*self.desti*wi)-(2*self.destj*wj)),
                                                #    self.nn.eq(self.nn+1)
                                                   ]
                                        with m.If(self.g>gw):
                                            m.d.sync+=[
                                            self.g.eq(gw),
                                            self.xi.eq(wi),
                                            self.xj.eq(wj)
                                            ]

                m.next=states.print
                #to check if it has valid no of neighbours
                # if cell has more than 2 cell as it valid neighbours then it is
                # valid
            with m.State(states.non):
                with m.If(self.nn<2):
                    m.next=states.End
                with m.Else():
                    m.next=states.print

            with m.State(states.print):
                m.d.sync+= self.st.eq(states.print)
                # storing the next cell values in i,j and temi,temj
                m.d.sync+=[
                        self.temi.eq(self.xi),
                        self.temj.eq(self.xj)
                        ]
                m.d.comb+=[
                    self.i.eq(self.xi),
                    self.j.eq(self.xj),
                    ]

                m.next=states.isdest

            with m.State(states.isdest):
                m.d.sync+=self.st.eq(states.isdest)
                # checking if it is i,j are destination or not
                # if it is destination go to end state
                # else go to neighbour neighbours state
                with m.If(self.temi==self.desti):
                    with m.If(self.temj==self.destj):
                        m.next=states.End
                        m.d.sync+=self.found.eq(1)
                    with m.Else():
                        m.next=states.neighbours
                with m.Else():
                    m.next=states.neighbours

            with m.State(states.End):
                with m.If(self.found==1):
                    m.d.sync+=self.var.eq(states.Found)
                with m.Else():
                    m.d.sync+=self.var.eq(states.cantfind)

if __name__=="__main__":
    from amaranth.cli import main
    m=Module()
    m.submodules.abc=a=abc()
    s=Simulator(a)
    def process():
        yield a.start.eq(1)
        yield a.srci.eq(0)
        yield a.srcj.eq(3)
        yield a.desti.eq(3)
        yield a.destj.eq(1)
        for _  in range(50):
            yield
    s.add_clock(1e-6)
    s.add_sync_process(process)

    with s.write_vcd("acc.vcd","acc.gtkw",traces=a.ports()):
        s.run()
    main(m,ports=a.ports())
    # main(a,ports=a.ports())