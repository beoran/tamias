package tamias


type ArrayElement interface {} 

type Array struct{
  num, max int
  arr []ArrayElement
} 

func ArrayAlloc() (*Array) {
  return &Array{}
}


func (arr * Array) Init(size int)  (*Array) {
  arr.num = 0
  if size < 4 { size    =  4; }
  arr.max = size
  arr.arr = make([]ArrayElement, size)
  return arr
}

func ArrayNew(size int)  (*Array) {
  return ArrayAlloc().Init(size)
}

func (arr * Array) Destroy()  {
  arr.arr = nil
}

func (arr * Array) Size() (int)  {
  return arr.num
}

func (arr * Array) Index(i int) (ArrayElement) {
  if i < 0 || i >= arr.max { return nil }
  return arr.arr[i]
}

func (arr * Array) Free() {
  arr.Destroy()
}

func (arr * Array) Push(object ArrayElement)  {
  if(arr.num == arr.max){
    arr.max *= 2
    newarr  := make([]ArrayElement, arr.max)
    // copy old to new 
    copy(newarr, arr.arr) 
    arr.arr = newarr
  }
  arr.arr[arr.num] = object
  arr.num++  
}

func (arr * Array) Pop() (object ArrayElement) {
  arr.num--
  
  value := arr.arr[arr.num]
  arr.arr[arr.num] = nil  
  
  return value
}  

func (arr * Array) DeleteIndex(idx int) { 
  arr.num--  
  arr.arr[idx]      = arr.arr[arr.num]
  arr.arr[arr.num]  = nil
}

func (arr * Array) DeleteObj(obj ArrayElement) { 
  for i:=0; i<arr.num; i++ {
    if arr.arr[i] == obj {
      arr.DeleteIndex(i)
      return
    }
  }
}

/*
void
cpArrayEach(cpArray *arr, cpArrayIter iterFunc, void *data)
{
  for(int i=0; i<arr.num; i++)
    iterFunc(arr.arr[i], data)
}
*/

func (arr * Array) Contains(obj ArrayElement) (bool) { 
  for i:=0; i<arr.num; i++ {
    if arr.arr[i] == obj {      
      return true
    }
  }
  return false;
}

