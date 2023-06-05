#include <iostream>
using namespace std;
 
enum Direction
{
    North,
	South,
	East,
	West	
};
 
class Point
{
	public:
		int x;
		int y;
		Point(const Point &point)
        {
        	x = point.x;
        	y = point.y;
        } 
 
		Point(int x,int y)
        {
        	this->x = x;
        	this->y = y;
        }
        Point()
        {
		} 
		~Point(){}
    	bool operator==(Point& other)
    	{
     		return this->x == other.x && this->y == other.y;
    	}
	
    	bool operator!=(Point& other)
    	{
	    	return this->x != other.x || this->y != other.y;
    	}
	
     	Point operator=(Point& other)
	    {
            this->x = other.x;
            this->y = other.y;
            return *this;
    	}
		
};
 
class PointList
{ 
    public:
	    Point point;
		PointList* next;
		void Insert(Point point);
		friend bool Delete(PointList*& tar, Point point);
		void Fix(Point point,Point tar);
		bool Search(Point point);
			
};
 
class Rectangle
{
	public:
		int xMin;
		int xMax;
		int yMin;
		int yMax;
		Rectangle(int xMin,int xMax,int yMin,int yMax)
		{
			this->xMin = xMin;
			this->xMax = xMax;
			this->yMin = yMin;
			this->yMax = yMax;
		}
		
		bool operator==(const Rectangle& other)
		{
			return this->xMin == other.xMin && this->xMax == other.xMax && this->yMin == other.yMin && this->yMax == other.yMax;
		}
		
		bool operator!=(const Rectangle& other)
		{
			return !(this->xMin == other.xMin && this->xMax == other.xMax && this->yMin == other.yMin && this->yMax == other.yMax);
		}
		
	    friend ostream& operator<<(ostream& output,Rectangle rec);
		
		bool IsInRectangle(Point tar);
		bool Contains(Rectangle tar);
		Rectangle* Split();
		Rectangle(){}
		~Rectangle(){}
};
 
class QuadTree
{
	public:
		Rectangle treeRec;
		
		QuadTree(){}
		QuadTree(Rectangle treeRec){this->treeRec = treeRec;}
		QuadTree(Rectangle treeRec,PointList* pointList)
		{
			this->treeRec = treeRec;
			this->pointList = pointList; 
		}
		
		QuadTree(Rectangle treeRec,QuadTree* parent)
		{
			this->treeRec = treeRec;
			this->pointList = pointList; 
			this->parent = parent;
			
			PointList* parentList = parent->pointList;
			pointList = new PointList();
			Point p(-1,-1);
			pointList->point = p;
			//cout<<pointList->point.x <<endl; 
			pointList->next = NULL;
			while(parentList!=NULL)
			{
				if(treeRec.IsInRectangle(parentList->point))
				{
					if(pointList->point.x == -1)
					{
						pointList->point = parentList->point;
						//cout<<1<<endl;
					}
					else
					{
						pointList->Insert(parentList->point);
					}
				}
				
				parentList = parentList->next;
			}
			
		}  
		~QuadTree()
		{
			delete rightTopTree;
			delete leftTopTree;
			delete leftBottomTree;
			delete rightBottomTree;
			delete pointList;
			delete this;
		}
		
		QuadTree* rightTopTree;
		QuadTree* leftTopTree;
		QuadTree* leftBottomTree;
		QuadTree* rightBottomTree;
		
		QuadTree* parent;
		
		PointList* pointList;
		
		void CreateChildren();
		
		Rectangle* RectangleSearch(Point point,Rectangle* rec,QuadTree* root);
		
		bool PointSearch(Point point);
		
		bool PointDelete(Point point,QuadTree* tar);
		
		bool RectangleDelete(Rectangle rec,QuadTree* tar);
		
		void DeleteAllChildren(QuadTree* tar);
		
		QuadTree* FindNeibor(Direction dir,QuadTree* tar,QuadTree* parent); 
		
};
 
//int main()
//{
//	Rectangle* rootRec = new Rectangle(0,16,0,16);
//	QuadTree* root = new QuadTree(*rootRec);
//	root->pointList = new PointList();
//	Point rootPoint(1,2);
//	root->pointList->point = rootPoint;
//	root->parent = NULL;
//	root->pointList->next = NULL;
//	root->pointList->Insert(Point(2,9));
//	root->pointList->Insert(Point(2,14));
//	root->pointList->Insert(Point(5,9));
//	root->pointList->Insert(Point(5,11));
//	root->pointList->Insert(Point(6,14));
//	root->pointList->Insert(Point(7,9));
//	root->pointList->Insert(Point(7,11));
//	root->pointList->Insert(Point(9,6));
//	root->pointList->Insert(Point(9,12));
//	root->CreateChildren();
//	root->leftTopTree->CreateChildren();
//	root->leftTopTree->rightBottomTree->CreateChildren();
//	root->leftTopTree->rightTopTree->CreateChildren();
//	
//	Rectangle* test = new Rectangle();
//	test->xMin = 4;
//	test->xMax = 6;
//	test->yMin = 8;
//	test->yMax = 10;
//	
//	cout<<root->leftTopTree->rightBottomTree->leftTopTree->FindNeibor(North,root->leftTopTree->rightBottomTree->leftTopTree,root->leftTopTree->rightBottomTree)->treeRec;
//	
//	return 0;
//}
/*
找一个节点的北方邻居：如果这个节点没有父节点，则它没有北方邻居。如果它有父节点，如果它是父节点的左下或者右下孩子，则它的北方邻居为它正上方与它
同一层的节点；如果它是父节点下左上或者右上孩子，则需要按如下方式递归：递归的找到它父级节点的北方邻居μ，如果μ有子节点，则它的北方邻居是μ的子节点，否则为μ本身。 
其他方向的邻居同理。 
*/
 
QuadTree* QuadTree::FindNeibor(Direction dir,QuadTree* tar,QuadTree* parent)
{
	switch(dir)
	{
		if(parent == NULL)
			return NULL;
		QuadTree* parentNeibor;
		case North:
			if(tar == parent->leftBottomTree)
			    return parent->leftTopTree;
			if(tar == parent->rightBottomTree)
			    return parent->rightTopTree;
		    //找到它父节点的北方邻居 
            parentNeibor = FindNeibor(North,parent,parent->parent);
            //父节点没有孩子节点 
            if(parentNeibor->rightTopTree == NULL)
                return parentNeibor;
            //父节点有孩子节点
			if(tar == parent->leftTopTree)
			    return parentNeibor->leftBottomTree;
			return parentNeibor->rightBottomTree; 
			break;
			
		case South:
			if(tar == parent->leftTopTree)
			    return parent->leftBottomTree;
			if(tar == parent->rightTopTree)
			    return parent->rightBottomTree;
            parentNeibor = FindNeibor(South,parent,parent->parent);
            if(parentNeibor->rightTopTree == NULL)
                return parentNeibor;
            if(tar == parent->leftBottomTree)
                return parentNeibor->leftTopTree;
            return parentNeibor->rightTopTree;
			
			break;
			
		case East:
			if(tar == parent->leftTopTree)
			    return parent->rightTopTree;
			if(tar == parent->leftBottomTree)
			    return parent->rightBottomTree;
            parentNeibor = FindNeibor(East,parent,parent->parent);
			if(tar == parent->rightTopTree)
			    return parentNeibor->leftTopTree;
			return parentNeibor->leftBottomTree;
		    break;
		    
		case West:
			if(tar == parent->rightTopTree)
			    return parent->leftTopTree;
			if(tar == parent->rightBottomTree)
			    return parent->leftBottomTree;
            parentNeibor = FindNeibor(West,parent,parent->parent);
            if(tar == parent->leftTopTree)
                return parentNeibor->rightTopTree;
            return parentNeibor->rightBottomTree;
			break;
	}
	return NULL;	
}
 
ostream& operator<<(ostream& output,Rectangle rec)
{
	output<<rec.xMin <<" "<<rec.xMax<<" "<<rec.yMin<<" "<<rec.yMax<<endl;
	return output;
}
 
 
bool QuadTree::PointDelete(Point point,QuadTree* tar)
{
	bool deleteSuccess = Delete(tar->pointList,point);
	if(!deleteSuccess)
	    return false;
	if(!tar->rightTopTree)
	    return true;
	if(tar->rightTopTree->treeRec.IsInRectangle(point))
	{
		return PointDelete(point,tar->rightTopTree);
	}
	
	if(tar->leftTopTree->treeRec.IsInRectangle(point))
	{
		return PointDelete(point,tar->leftTopTree);
	}
	
	if(tar->leftBottomTree->treeRec.IsInRectangle(point))
	{
		return PointDelete(point,tar->leftBottomTree);
	}
	
	if(tar->rightBottomTree->treeRec.IsInRectangle(point))
	{
		return PointDelete(point,tar->rightBottomTree);
	}
}
 
void QuadTree::DeleteAllChildren(QuadTree* tar)
{
	QuadTree* rightTop = tar->rightTopTree;
	QuadTree* leftTop = tar->leftTopTree;
	QuadTree* leftBottom = tar->leftBottomTree;
	QuadTree* rightBottom = tar->rightBottomTree;
	
	tar->rightTopTree = NULL;
	tar->leftTopTree = NULL;
	tar->leftBottomTree = NULL;
	tar->rightBottomTree = NULL;
}
 
bool QuadTree::RectangleDelete(Rectangle rec,QuadTree* tar)
{
	if(!tar->treeRec.Contains(rec))
	{
	    return false;
	}
	 
	if(tar->rightTopTree&&tar->rightTopTree->treeRec.Contains(rec))
	{
		if(tar->rightTopTree->treeRec == rec)
		{
			DeleteAllChildren(tar);
			return true;
		}
		return RectangleDelete(rec,tar->rightTopTree);
	}
	
	if(tar->leftTopTree&&tar->leftTopTree->treeRec.Contains(rec))
	{
		if(tar->leftTopTree->treeRec == rec)
		{
		    DeleteAllChildren(tar);
			return true;
		}
		return RectangleDelete(rec,tar->leftTopTree);
	}
	
	if(tar->leftBottomTree&&tar->leftBottomTree->treeRec.Contains(rec))
	{
		if(tar->leftBottomTree->treeRec == rec)
		{
		    DeleteAllChildren(tar);
			return true;
		}
		return RectangleDelete(rec,tar->leftBottomTree);
	}
	
	if(tar->rightBottomTree&&tar->rightBottomTree->treeRec.Contains(rec))
	{
		if(tar->rightBottomTree->treeRec == rec)
		{
	        DeleteAllChildren(tar);
			return true;
		}
		return RectangleDelete(rec,tar->rightBottomTree);
	}
}
 
bool Rectangle::IsInRectangle(Point tar)
{
	return tar.x > xMin && tar.x < xMax && tar.y > yMin && tar.y < yMax;
}
 
bool Rectangle::Contains(Rectangle tar)
{
	return this->xMin <= tar.xMin && this->xMax >= tar.xMax && this->yMin <= tar.yMin && this->yMax >= tar.yMax;
}
 
bool QuadTree::PointSearch(Point point)
{
	return this->pointList->Search(point);
}
 
Rectangle* QuadTree::RectangleSearch(Point point,Rectangle* rec,QuadTree* root)
{
	if(!rec->IsInRectangle(point))
	{
		return NULL;
	}
	
	if(root->rightTopTree&&root->rightTopTree->treeRec.IsInRectangle(point))
	{
		return QuadTree::RectangleSearch(point,&root->rightTopTree->treeRec,root->rightTopTree);
	}
	
	if(root->leftTopTree&&root->leftTopTree->treeRec.IsInRectangle(point))
	{
		return QuadTree::RectangleSearch(point,&root->leftTopTree->treeRec,root->leftTopTree);
	} 
	
	if(root->leftBottomTree&&root->leftBottomTree->treeRec.IsInRectangle(point))
	{
		return QuadTree::RectangleSearch(point,&root->leftBottomTree->treeRec,root->leftBottomTree);
	}
	
	if(root->rightBottomTree&&root->rightBottomTree->treeRec.IsInRectangle(point))
	{
		return QuadTree::RectangleSearch(point,&root->rightBottomTree->treeRec,root->rightBottomTree);
	}
	
	return rec;
	
}
 
bool PointList::Search(Point point)
{
	PointList* hip = this;
	while(hip != NULL)
	{
		if(hip->point == point)
		    return true;
		hip = hip->next;
	}
	return false;
}
 
void PointList::Fix(Point point,Point tar)
{
	PointList* hip = this;
	while(hip->next->point != tar)
	{
		hip = hip->next;
	}
	hip->next->	point = point;
}
 
 
void PointList::Insert(Point point)
{
	PointList* hip = this;
	while(hip->next)
	{
		hip = hip->next;
	}
	PointList* nextPoint = new PointList();
	nextPoint->point = point;
	nextPoint->next = NULL;
	hip->next = nextPoint;
}
 
bool Delete(PointList*& tar,Point point)
{
	PointList* hip = tar;
	
	if(hip->point == point)
	{
		tar = hip->next;
		delete hip;
		return true;
	}
			
	while(hip->next && hip->next->point != point)
	{
		hip = hip->next;
	}
	
	if(!hip->next)
	    return false;
	
	PointList* pointToBeDeleted = hip->next;
	hip->next = hip->next->next;
	delete pointToBeDeleted;
	return true;
}
 
Rectangle* Rectangle::Split()
{
	Rectangle* rec = new Rectangle[4];
	rec[0].xMin = (this->xMin + this->xMax)/2;
	rec[0].xMax = this->xMax;
	rec[0].yMin = (this->yMin + this->yMax)/2;
	rec[0].yMax = this->yMax;
	
	rec[1].xMin = this->xMin;
	rec[1].xMax = (this->xMin + this->xMax)/2;
	rec[1].yMin = (this->yMin + this->yMax)/2;
	rec[1].yMax = this->yMax;
	
	rec[2].xMin = this->xMin;
	rec[2].xMax = (this->xMin + this->xMax)/2;
	rec[2].yMin = this->yMin;
	rec[2].yMax = (this->yMin + this->yMax)/2;
	
	rec[3].xMin = (this->xMin + this->xMax)/2;
	rec[3].xMax = this->xMax;
	rec[3].yMin = this->yMin;
	rec[3].yMax = (this->yMin + this->yMax)/2;
	return rec;
	
}
 
void QuadTree::CreateChildren()
{
	Rectangle* tarRecArray = this->treeRec.Split();
	
	rightTopTree = new QuadTree(tarRecArray[0],this);
	
	leftTopTree = new QuadTree(tarRecArray[1],this);
	
	leftBottomTree = new QuadTree(tarRecArray[2],this);
	
	rightBottomTree = new QuadTree(tarRecArray[3],this);
	
	
}