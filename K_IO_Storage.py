import sys, getopt



if __name__ == "__main__":
    print(" Number of argumentst ", len(sys.argv))

    i = 0
    for argument in sys.argv:
       print("     ", i, "  ", argument)

'''
   inputfile = None
   outputfile = None


   try :
      opts, args = getopt.getopt(sys.argv,"hi:o:",["ifile=","ofile="])
   except getopt.GetoptError:
      print( 'test.py -i <inputfile> -o <outputfile>')
      sys.exit(2)

   for opt, arg in opts:
      if opt == '-h':
         print('test.py -i <inputfile> -o <outputfile>')
         sys.exit()
      elif opt in ("-i", "--ifile"):
         inputfile = arg
      elif opt in ("-o", "--ofile"):
         outputfile = arg
   print('Input file is "', inputfile)
   print('Output file is "', outputfile)

 

'''