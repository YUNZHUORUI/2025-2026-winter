def printarray(arr,format1,**kwargs):
    '''
    Function to format and print a 1D array.
      arr: the array
      format1: a string with the required format, e.g., .5e for exponential with 5 decimal digits
      kwargs: name: a string with the name of the array in the calling program, to be prefixed
              eu: engineeering units, to be postfixed
    '''

    # print prefix if any
    if 'name' in kwargs:
        print(kwargs['name'],' = [',end="")
    else:
        print('[',end="")
    
    # print array
    nn = arr.size
    fm = '{:'+format1+'} '
    for ii in range(nn):print(fm.format(arr[ii]),end="")
    print(']',end="")
    
    # print units if any
    if 'eu' in kwargs:
        print(' ',kwargs['eu'])
    else:
        print('')
        
    return
    
    
    
def writearray(studfile,arr,format1,**kwargs):
    '''
    Function to format and write to a file a 1D array.
      studfile: string; filename of file where array is to be appended
      arr: the array
      format1: a string with the required format, e.g., .5e for exponential with 5 decimal digits
      kwargs: name: a string with the name of the array in the calling program, to be prefixed
              eu: engineeering units, to be postfixed
              blankline: Bolean; if True a linefeed character is written at the end of the line
              to create a blank line following the array
    '''
    with open(studfile, 'a') as fi:
 
        # write prefix if any
        if 'name' in kwargs:
            prnstr = '{} = ['
            fi.write(prnstr.format(kwargs['name']))
        else:
            prnstr = '['
            fi.write(prnstr)
        
        # write array
        nn = arr.size
        fm = '{:'+format1+'} '
        for ii in range(nn):fi.write(fm.format(arr[ii]))
            
        prnstr = ']'
        fi.write(prnstr)
        
        # write units if any
        if 'eu' in kwargs:
            prnstr = '{}\n'
            fi.write(prnstr.format(kwargs['eu']))
        else:
            fi.write('\n')

        if 'blankline' in kwargs:
            if 'blankline': fi.write('\n')

    fi.close()
        
    return    