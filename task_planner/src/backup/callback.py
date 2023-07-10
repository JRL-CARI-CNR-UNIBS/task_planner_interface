   # if where == gp.GRB.Callback.POLLING:
    #     # Ignore polling callback
    #     pass
    # elif where == gp.GRB.Callback.PRESOLVE:
    #     # Presolve callback
    #     cdels = model.cbGet(gp.GRB.Callback.PRE_COLDEL)
    #     rdels = model.cbGet(gp.GRB.Callback.PRE_ROWDEL)
    #     if cdels or rdels:
    #         print('%d columns and %d rows are removed' % (cdels, rdels))
    # elif where == gp.GRB.Callback.SIMPLEX:
    #     # Simplex callback
    #     itcnt = model.cbGet(gp.GRB.Callback.SPX_ITRCNT)
    #     if itcnt - model._lastiter >= 100:
    #         model._lastiter = itcnt
    #         obj = model.cbGet(gp.GRB.Callback.SPX_OBJVAL)
    #         ispert = model.cbGet(gp.GRB.Callback.SPX_ISPERT)
    #         pinf = model.cbGet(gp.GRB.Callback.SPX_PRIMINF)
    #         dinf = model.cbGet(gp.GRB.Callback.SPX_DUALINF)
    #         if ispert == 0:
    #             ch = ' '
    #         elif ispert == 1:
    #             ch = 'S'
    #         else:
    #             ch = 'P'
    #         print('%d %g%s %g %g' % (int(itcnt), obj, ch, pinf, dinf))
    # elif where == gp.GRB.Callback.MIP:
    #     # General MIP callback
    #     nodecnt = model.cbGet(gp.GRB.Callback.MIP_NODCNT)
    #     objbst = model.cbGet(gp.GRB.Callback.MIP_OBJBST)
    #     objbnd = model.cbGet(gp.GRB.Callback.MIP_OBJBND)
    #     solcnt = model.cbGet(gp.GRB.Callback.MIP_SOLCNT)
    #     # if nodecnt - model._lastnode >= 100:
    #     #     model._lastnode = nodecnt
    #     #     actnodes = model.cbGet(gp.GRB.Callback.MIP_NODLFT)
    #     #     itcnt = model.cbGet(gp.GRB.Callback.MIP_ITRCNT)
    #     #     cutcnt = model.cbGet(gp.GRB.Callback.MIP_CUTCNT)
    #     #     print('%d %d %d %g %g %d %d' % (nodecnt, actnodes,
    #     #                                     itcnt, objbst, objbnd, solcnt, cutcnt))
    #     # if abs(objbst - objbnd) < 0.1 * (1.0 + abs(objbst)):
    #     #     print('Stop early - 10% gap achieved')
    #     #     model.terminate()
    #     # if nodecnt >= 10000 and solcnt:
    #     #     print('Stop early - 10000 nodes explored')
    #     #     model.terminate()
    # elif where == gp.GRB.Callback.MIPSOL:
    #     # MIP solution callback
    #     nodecnt = model.cbGet(gp.GRB.Callback.MIPSOL_NODCNT)
    #     obj = model.cbGet(gp.GRB.Callback.MIPSOL_OBJ)
    #     solcnt = model.cbGet(gp.GRB.Callback.MIPSOL_SOLCNT)
    #     x = model.cbGetSolution(model.getVars())
    #     print('**** New solution at node %d, obj %g, sol %d, '
    #           'x[0] = %g ****' % (nodecnt, obj, solcnt, x[0]))
    #     print(model.cbGetSolution(model.getVars()))
    # 
    # elif where == gp.GRB.Callback.MIPNODE:
    #     # MIP node callback
    #     print('**** New node ****')c
    #     if model.cbGet(gp.GRB.Callback.MIPNODE_STATUS) == gp.GRB.OPTIMAL:
    #         x = model.cbGetNodeRel(model._vars)
    #         model.cbSetSolution(model.getVars(), x)
    # elif where == gp.GRB.Callback.BARRIER:
    #     # Barrier callback
    #     itcnt = model.cbGet(gp.GRB.Callback.BARRIER_ITRCNT)
    #     primobj = model.cbGet(gp.GRB.Callback.BARRIER_PRIMOBJ)
    #     dualobj = model.cbGet(gp.GRB.Callback.BARRIER_DUALOBJ)
    #     priminf = model.cbGet(gp.GRB.Callback.BARRIER_PRIMINF)
    #     dualinf = model.cbGet(gp.GRB.Callback.BARRIER_DUALINF)
    #     cmpl = model.cbGet(gp.GRB.Callback.BARRIER_COMPL)
    #     print('%d %g %g %g %g %g' % (itcnt, primobj, dualobj,
    #                                  priminf, dualinf, cmpl))
    # elif where == gp.GRB.Callback.MESSAGE:
    #     # Message callback
    #     msg = model.cbGet(gp.GRB.Callback.MSG_STRING)