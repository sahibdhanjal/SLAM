all:
	@make -C lcmtypes --no-print-directory                                                        
	@make -C mobilebot --no-print-directory
	@make -C test_motors --no-print-directory
	@make -C optitrack/common --no-print-directory                                                
	@make -C optitrack --no-print-directory                                                      

lcmspy:	
	/bin/bash setenv.sh
	@make -C lcmtypes --no-print-directory                                                     
	@make -C java --no-print-directory

clean:
	@make -C lcmtypes -s clean                                                 
	@make -C mobilebot -s clean
	@make -C test_motors -s clean
	@make -C optitrack/common -s clean                                         
	@make -C optitrack -s clean                                                
	@make -C java -s clean
