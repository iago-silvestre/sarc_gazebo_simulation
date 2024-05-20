package embedded.mas.bridges.jacamo;

import embedded.mas.bridges.jacamo.DefaultEmbeddedAgArch;
import embedded.mas.bridges.jacamo.IDevice;
import embedded.mas.exception.PerceivingException;

import jason.asSyntax.Literal;

import java.util.Collection;

public class CriticalEmbeddedAgArch extends DefaultEmbeddedAgArch{

	private int lastCPval = -12345; //LBB: this is temporary, requires a better implementation

    public CriticalEmbeddedAgArch() {
		super();
	}
				
    @Override
	public Boolean[] perceiveCBS() { //v0: Still not generic, works only for the hand-coded perception "value2"
		Boolean[] percepts = new Boolean[8];
		for(int i=0; i<8; i++)
			percepts[i] = Boolean.FALSE;
		
		for(IDevice s:this.devices) { //for each sensor
			try {
				Collection<Literal> p = s.getPercepts(); //List<Literal> beliefs = microcontroller.read(); return beliefs;
				if(p!=null) {
					for(Literal l:p) {
						String functor = l.getFunctor();
						if(functor.contains("critical_percept")){
							int value = Integer.parseInt(l.getTerm(0).toString());
							if(value != lastCPval){
								lastCPval = value;
								percepts[0] = Boolean.TRUE;
							}
							// else
							// 	percepts[0] = Boolean.FALSE;
						}
							
					}
				}
			} catch (PerceivingException e) {} //if it fails, do nothing 			
		}
		return percepts;
   	}
  
	
}